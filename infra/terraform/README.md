# Xen CI — GitLab Runner Autoscaler on AWS

Ephemeral GitLab runners on AWS Spot instances using the `docker-autoscaler` executor
and the [cattle-ops/terraform-aws-gitlab-runner](https://github.com/cattle-ops/terraform-aws-gitlab-runner) module (v9.x).

## Architecture

```
GitLab Server (gitpct.epam.com)
       │
       ▼ HTTPS
┌─────────────────────────────────────────────┐
│  Default VPC (eu-central-1)                 │
│                                             │
│  ┌──────────────┐                           │
│  │ Manager      │  t4g.nano (always-on)     │
│  │ gitlab-runner│  polls GitLab for jobs    │
│  └──────┬───────┘                           │
│         │ fleeting plugin                   │
│  ┌──────▼──────┐                            │
│  │ Worker ASG  │  Spot, on-demand only      │
│  │ m7g.8xl    │  50 jobs per instance      │
│  │ max: 2      │  idle 3 min then terminate │
│  └─────────────┘                            │
│                                             │
│  S3 cache bucket                            │
└─────────────────────────────────────────────┘
```

## Usage

### Prerequisites

- AWS CLI configured with permissions (EC2, ASG, IAM, S3, SSM, Lambda, CloudWatch)
- Terraform >= 1.5
- Terragrunt >= 0.69
- GitLab runner registration token stored in SSM Parameter Store

### Deploy

```bash
cd infra/terraform
terragrunt apply -auto-approve
```

### Destroy

```bash
cd infra/terraform
terragrunt destroy -auto-approve
```

Note: Lambda VPC ENIs take up to 20 minutes to detach after destroy. Be patient.

## How to Add a New Runner

1. Go to GitLab: Settings → CI/CD → Runners → "New project runner"
2. Set tags (e.g., `aws-arm64`), uncheck "Run untagged jobs"
3. Copy the `glrt-` token
4. Store in SSM:
   ```bash
   aws ssm put-parameter --region eu-central-1 \
     --name "/xen-ci/runner-token-arm64" \
     --value "glrt-YOUR_TOKEN" \
     --type SecureString --overwrite
   ```
5. Reference the SSM parameter name in the Terraform config:
   ```hcl
   runner_gitlab = {
     preregistered_runner_token_ssm_parameter_name = "/xen-ci/runner-token-arm64"
   }
   ```
6. `terragrunt apply`

## Concurrency Parameters

| Parameter | Where | What it controls |
|---|---|---|
| `maximum_concurrent_jobs` | `runner_manager` block | How many jobs the manager accepts from GitLab at once (`concurrent` in config.toml) |
| `max_jobs` | `runner_worker` block | Max worker instances in ASG (`max_size`). Also sets `limit` in config.toml as `max_jobs × capacity_per_instance` |
| `capacity_per_instance` | `runner_worker_docker_autoscaler` block | How many jobs run in parallel on a single worker instance (separate Docker containers) |
| `max_use_count` | `runner_worker_docker_autoscaler` block | Total jobs an instance handles before being replaced (fresh instance) |

### Example: 50 parallel jobs on 2 instances

```hcl
runner_manager = {
  maximum_concurrent_jobs = 50   # accept 50 jobs from GitLab
}

runner_worker = {
  type     = "docker-autoscaler"
  max_jobs = 2                   # max 2 worker instances
}

runner_worker_docker_autoscaler = {
  capacity_per_instance = 50     # 50 jobs per instance
  max_use_count         = 200    # replace instance after 200 total jobs
}
```

Total capacity: `max_jobs × capacity_per_instance` = 2 × 50 = 100 concurrent jobs.

## Instance Size

Set in `variables.tf`:

```hcl
variable "worker_arm64_instance_types" {
  default = ["c7g.16xlarge", "c6g.16xlarge"]  # 64 vCPU, 128 GB RAM
}
```

Choose based on `capacity_per_instance`:
- 4 jobs/instance → `m7g.xlarge` (4 vCPU, 16GB)
- 10 jobs/instance → `m7g.2xlarge` (8 vCPU, 32GB)
- 50 jobs/instance → `m7g.8xlarge` (32 vCPU, 128GB)

Rule of thumb: ~1 vCPU + ~2GB RAM per concurrent build job.

## Worker Idle Time

Workers stay alive for 3 minutes after last job, waiting for new work:

```hcl
runner_worker_docker_machine_instance = {
  idle_count = 0
  idle_time  = 180  # seconds
}
```

Set to `0` for immediate termination (slower cold starts).

## Docker Pull Policy

Set to `if-not-present` — first job pulls image, subsequent jobs use cache:

```hcl
runner_worker_docker_options = {
  pull_policies = ["if-not-present"]
}
```

## CI Job Tags

Jobs must have the correct tag to match the runner:

| Tag | Runner |
|---|---|
| `aws-arm64` | ARM64 Graviton workers |
| `aws-x86` | x86_64 workers (currently disabled) |

Hardware test runners (`xilinx`, `qubes-hw*`, `epdefrans`) are unchanged and run on-prem.

## Troubleshooting

### Jobs stuck in "Preparing executor"
- Worker instance booting + installing Docker (~90s)
- Check: `aws autoscaling describe-auto-scaling-groups --auto-scaling-group-names ci-arm-asg`

### "docker: command not found"
- Docker install hasn't finished. Worker start_script installs Docker before enabling SSH.
- If this persists, the start_script failed. Check instance console output.

### Manager instances multiplying
- Normal during `terragrunt apply` — rolling refresh replaces old with new.
- Old instance terminates after 30s lifecycle hook timeout.

### Runner not registering (403 Forbidden)
- Token expired or invalid. Create new runner on GitLab, update SSM parameter, terminate manager instance.

### Worker ASG max_size = 0
- Module v7.x bug. Use v9.x+ which doesn't ignore `max_size` changes.
