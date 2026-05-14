module "gitlab_runner_arm64" {
  source  = "cattle-ops/gitlab-runner/aws"
  version = "~> 9.0"

  environment = "ci-arm"

  vpc_id    = data.aws_vpc.default.id
  subnet_id = data.aws_subnets.default.ids[1]

  # Runner manager instance
  runner_instance = {
    name                 = "xen-ci-manager-arm64"
    type                 = "t4g.nano"
    private_address_only = false
  }

  runner_ami_filter = {
    name = ["al2023-ami-2023*-arm64"]
  }
  runner_ami_owners = ["amazon"]

  # GitLab connection
  runner_gitlab = {
    url            = var.gitlab_url
    runner_version = "17.4.0"
    preregistered_runner_token_ssm_parameter_name = "/xen-ci/runner-token-arm64"
  }

  runner_manager = {
    maximum_concurrent_jobs = 50
  }

  runner_worker = {
    type     = "docker-autoscaler"
    max_jobs = 2
  }

  runner_worker_docker_options = {
    privileged   = true
    volumes      = ["/cache", "/var/run/docker.sock:/var/run/docker.sock"]
    image        = "alpine:latest"
    pull_policies = ["if-not-present"]
  }

  # Fleeting plugin (docker-autoscaler)
  runner_worker_docker_autoscaler = {
    fleeting_plugin_version = "1.1.0"
    connector_config_user   = "ubuntu"
    max_use_count           = 10
    capacity_per_instance   = 50
  }

  runner_worker_docker_autoscaler_ami_filter = {
    name = ["ubuntu/images/hvm-ssd-gp3/ubuntu-noble-24.04-arm64-server-*"]
  }
  runner_worker_docker_autoscaler_ami_owners = ["099720109477"]

  runner_worker_docker_autoscaler_instance = {
    root_size            = var.worker_root_volume_size
    private_address_only = false
    start_script         = <<-EOF
      #!/bin/bash
      set -e
      export DEBIAN_FRONTEND=noninteractive
      # Stop SSH until Docker is ready
      systemctl stop ssh.socket ssh.service 2>/dev/null || true
      # Install Docker from official repo
      apt-get update -qq
      apt-get install -y -qq ca-certificates curl >/dev/null 2>&1
      install -m 0755 -d /etc/apt/keyrings
      curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" > /etc/apt/sources.list.d/docker.list
      apt-get update -qq
      apt-get install -y -qq docker-ce docker-ce-cli containerd.io >/dev/null 2>&1
      systemctl enable docker
      systemctl start docker
      usermod -aG docker ubuntu
      # Re-enable SSH now that Docker is ready
      systemctl start ssh.socket ssh.service
    EOF
  }

  runner_worker_docker_autoscaler_asg = {
    types                                    = var.worker_arm64_instance_types
    enable_mixed_instances_policy            = true
    on_demand_base_capacity                  = 0
    on_demand_percentage_above_base_capacity = 0
    spot_allocation_strategy                 = "lowest-price"
    subnet_ids                               = data.aws_subnets.default.ids
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
