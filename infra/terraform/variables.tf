variable "aws_region" {
  description = "AWS region for all resources"
  type        = string
  default     = "eu-central-1"
}

variable "public_subnet_cidr" {
  description = "CIDR for the public subnet hosting the NAT gateway"
  type        = string
  default     = "172.31.96.0/24"
}

variable "private_subnet_cidr" {
  description = "CIDR for the private subnet where runners operate"
  type        = string
  default     = "172.31.97.0/24"
}

variable "environment" {
  description = "Environment name (e.g., ci, staging, prod)"
  type        = string
  default     = "ci"
}

variable "gitlab_url" {
  description = "GitLab instance URL"
  type        = string
  default     = "https://gitpct.epam.com"
}

variable "worker_arm64_instance_types" {
  description = "Instance types for arm64 Spot workers (priority order)"
  type        = list(string)
  default     = ["m7g.8xlarge", "m6g.8xlarge"]
}

variable "worker_max_count_arm64" {
  description = "Max concurrent arm64 worker instances"
  type        = number
  default     = 2
}

variable "worker_x86_instance_types" {
  description = "Instance types for x86_64 Spot workers (priority order)"
  type        = list(string)
  default     = ["c6i.16xlarge", "c6a.16xlarge", "c5.18xlarge"]
}

variable "worker_max_count_x86" {
  description = "Max concurrent x86_64 worker instances"
  type        = number
  default     = 2
}

variable "cache_bucket_prefix" {
  description = "Prefix for the S3 cache bucket name"
  type        = string
  default     = "xen-ci-runner-cache"
}

variable "worker_root_volume_size" {
  description = "Root volume size in GB for worker instances"
  type        = number
  default     = 50
}
