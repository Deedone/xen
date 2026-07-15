module "gitlab_runner_arm64" {
  source  = "cattle-ops/gitlab-runner/aws"
  version = "~> 9.0"

  environment = "ci-arm"

  vpc_id    = data.aws_vpc.default.id
  subnet_id = aws_subnet.private[0].id

  # Runner manager instance — private only, reaches GitLab via NAT gateway
  runner_instance = {
    name                 = "xen-ci-manager-arm64"
    type                 = "t4g.nano"
    private_address_only = true
  }

  runner_ami_filter = {
    name = ["al2023-ami-2023*-arm64"]
  }
  runner_ami_owners = ["amazon"]

  # GitLab connection
  runner_gitlab = {
    url                                           = var.gitlab_url
    runner_version                                = "17.4.0"
    preregistered_runner_token_ssm_parameter_name = "/xen-ci/runner-token-arm64"
  }

  runner_manager = {
    maximum_concurrent_jobs = 150
  }

  runner_worker = {
    type     = "docker-autoscaler"
    max_jobs = 2
  }

  runner_worker_docker_options = {
    privileged    = true
    volumes       = ["/cache", "/var/run/docker.sock:/var/run/docker.sock"]
    image         = "alpine:latest"
    pull_policies = ["if-not-present"]
  }

  # Fleeting plugin (docker-autoscaler)
  runner_worker_docker_autoscaler = {
    fleeting_plugin_version = "1.1.0"
    connector_config_user   = "ubuntu"
    max_use_count           = 1000
    capacity_per_instance   = 75
  }

  runner_worker_docker_autoscaler_ami_filter = {
    name = ["ubuntu/images/hvm-ssd-gp3/ubuntu-noble-24.04-arm64-server-*"]
  }
  runner_worker_docker_autoscaler_ami_owners = ["099720109477"]

  runner_worker_docker_autoscaler_instance = {
    root_size            = var.worker_root_volume_size
    private_address_only = true
    start_script         = <<-EOF
      #!/bin/bash
      export DEBIAN_FRONTEND=noninteractive

      # Ensure SSH is always re-enabled even if the script fails
      trap 'systemctl start ssh.socket ssh.service' EXIT

      # Stop SSH until Docker is ready (prevents runner from connecting too early)
      systemctl stop ssh.socket ssh.service 2>/dev/null || true

      # Install Docker from official repo with retries
      for attempt in 1 2 3; do
        apt-get update -qq && break
        echo "apt-get update failed (attempt $attempt/3), retrying in 10s..."
        sleep 10
      done

      apt-get install -y -qq ca-certificates curl >/dev/null 2>&1
      install -m 0755 -d /etc/apt/keyrings
      curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" > /etc/apt/sources.list.d/docker.list

      for attempt in 1 2 3; do
        apt-get update -qq && apt-get install -y -qq docker-ce docker-ce-cli containerd.io >/dev/null 2>&1 && break
        echo "Docker install failed (attempt $attempt/3), retrying in 10s..."
        sleep 10
      done

      systemctl enable docker
      systemctl start docker
      usermod -aG docker ubuntu

      # SSH re-enabled by EXIT trap, but explicit for clarity
      systemctl start ssh.socket ssh.service

      # Pre-pull images in background (jobs can start immediately, pulls happen in parallel)
      (docker pull xentroops/xen_rel;
       docker pull xentroops/xen_artifacts_rel) &
    EOF
  }

  runner_worker_docker_autoscaler_asg = {
    types                                    = var.worker_arm64_instance_types
    enable_mixed_instances_policy            = true
    on_demand_base_capacity                  = 0
    on_demand_percentage_above_base_capacity = 0
    spot_allocation_strategy                 = "price-capacity-optimized"
    spot_instance_pools                      = 0
    subnet_ids                               = aws_subnet.private[*].id
    upgrade_strategy                         = "off"
  }

  # Terminate old manager quickly on replacement
  runner_terminate_ec2_lifecycle_timeout_duration = 30

  runner_worker_docker_autoscaler_autoscaling_options = []

  runner_worker_docker_machine_instance = {
    idle_count = 0
    idle_time  = 180
  }

  # S3 cache
  runner_worker_cache = {
    create        = true
    bucket_prefix = "xen-ci-arm64"
  }

  tags = {
    Project     = "xen-ci"
    Environment = var.environment
    Arch        = "arm64"
    ManagedBy   = "terraform"
  }
}
