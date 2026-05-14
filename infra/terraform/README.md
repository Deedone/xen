# Xen CI — GitLab Runner Autoscaler on AWS

Terraform configuration for ephemeral GitLab runners on AWS Spot instances (arm64 only).

## Architecture

```
GitLab Server (epam)
       │
       ▼ HTTPS
┌─────────────────────────────────────────────┐
│  VPC: 10.0.0.0/16  (eu-central-1)          │
│                                             │
│  ┌──────────────┐                           │
│  │ Manager      │                           │
│  │ (t4g.micro)  │                           │
│  └──────┬───────┘                           │
│         │                                   │
│  ┌──────▼──────┐                            │
│  │ ASG: arm64  │                            │
│  │ Graviton    │                            │
│  │ c7g.4xlarge │                            │
│  │ max: 10     │                            │
│  │ idle: 0     │                            │
│  └─────────────┘                            │
│                                             │
│  S3 cache bucket                            │
└─────────────────────────────────────────────┘
```

## Usage

```bash
cp terraform.tfvars.example terraform.tfvars
# Edit terraform.tfvars with your values

terraform init
terraform plan
terraform apply
```

## Key design decisions

- **IdleCount = 0**: instances only launch for builds, zero cost when idle
- **No pre-warming**: strictly on-demand
- **MaxUseCount = 1**: fresh instance per job (clean environment)
- **100% Spot**: cost savings ~70%; multiple instance types for availability
- **arm64 only**: Graviton workers
- **S3 cache**: shared build cache between jobs

## Prerequisites

- AWS account with permissions for EC2, ASG, IAM, S3, VPC
- GitLab runner registration token (project or group level)
- Terraform >= 1.5
