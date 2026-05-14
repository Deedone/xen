module "gitlab_runner_x86" {
  source  = "cattle-ops/gitlab-runner/aws"
  version = "~> 7.0"

  environment = "ci-x86"

  vpc_id    = data.aws_vpc.default.id
  subnet_id = data.aws_subnets.default.ids[0]

  # Runner manager instance
  runner_instance = {
    name                 = "xen-ci-manager-x86"
    type                 = "t3.nano"
    private_address_only = false
  }

  runner_ami_filter = {
    name = ["al2023-ami-2023*-x86_64"]
  }
  runner_ami_owners = ["amazon"]

  # GitLab connection
  runner_gitlab = {
    url            = var.gitlab_url
    runner_version = "17.4.0"
    preregistered_runner_token_ssm_parameter_name = "/xen-ci/runner-token-x86"
  }

  runner_manager = {
    maximum_concurrent_jobs = var.worker_max_count_x86
  }

  runner_worker = {
    type     = "docker-autoscaler"
    max_jobs = var.worker_max_count_x86
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
    name = ["ubuntu/images/hvm-ssd-gp3/ubuntu-noble-24.04-amd64-server-*"]
  }
  runner_worker_docker_autoscaler_ami_owners = ["099720109477"]

  runner_worker_docker_autoscaler_instance = {
    root_size            = var.worker_root_volume_size
    private_address_only = false
  }

  runner_worker_docker_autoscaler_asg = {
    types                                    = var.worker_x86_instance_types
    enable_mixed_instances_policy            = true
    on_demand_base_capacity                  = 0
    on_demand_percentage_above_base_capacity = 0
    spot_allocation_strategy                 = "lowest-price"
    subnet_ids                               = data.aws_subnets.default.ids
  }

  # Terminate old manager quickly on replacement
  runner_terminate_ec2_lifecycle_timeout_duration = 60

  runner_worker_docker_autoscaler_autoscaling_options = []

  runner_worker_docker_machine_instance = {
    idle_count = 0
    idle_time  = 180
  }

  # S3 cache
  runner_worker_cache = {
    create        = true
    bucket_prefix = "xen-ci-x86"
  }

  tags = {
    Project     = "xen-ci"
    Environment = var.environment
    Arch        = "x86_64"
    ManagedBy   = "terraform"
  }
}
