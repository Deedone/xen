output "vpc_id" {
  description = "VPC ID"
  value       = data.aws_vpc.default.id
}

output "cache_bucket_arm64" {
  description = "S3 cache bucket for arm64 runners"
  value       = module.gitlab_runner_arm64.runner_cache_bucket_name
}

output "cache_bucket_x86" {
  description = "S3 cache bucket for x86 runners"
  value       = module.gitlab_runner_x86.runner_cache_bucket_name
}
