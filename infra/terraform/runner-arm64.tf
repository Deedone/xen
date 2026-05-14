module "gitlab_runner_arm64" {
  source  = "cattle-ops/gitlab-runner/aws"
  version = "~> 7.0"

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
    maximum_concurrent_jobs = var.worker_max_count_arm64
  }

  runner_worker = {
    type = "docker-autoscaler"
  }

  runner_worker_docker_options = {
    privileged = true
    volumes    = ["/cache", "/var/run/docker.sock:/var/run/docker.sock"]
    image      = "alpine:latest"
  }

  # Fleeting plugin (docker-autoscaler)
  runner_worker_docker_autoscaler = {
    fleeting_plugin_version = "1.1.0"
    connector_config_user   = "ubuntu"
    max_use_count           = 1
  }

  runner_worker_docker_autoscaler_ami_filter = {
    name = ["ubuntu/images/hvm-ssd-gp3/ubuntu-noble-24.04-arm64-server-*"]
  }
  runner_worker_docker_autoscaler_ami_owners = ["099720109477"]

  runner_worker_docker_autoscaler_instance = {
    root_size            = var.worker_root_volume_size
    private_address_only = false
  }

  runner_worker_docker_autoscaler_asg = {
    types                                    = var.worker_arm64_instance_types
    enable_mixed_instances_policy            = true
    on_demand_base_capacity                  = 0
    on_demand_percentage_above_base_capacity = 0
    spot_allocation_strategy                 = "lowest-price"
    subnet_ids                               = data.aws_subnets.default.ids
  }

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
