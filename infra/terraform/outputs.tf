output "vpc_id" {
  description = "VPC ID (default VPC)"
  value       = data.aws_vpc.default.id
}

output "nat_gateway_public_ip" {
  description = "Fixed outbound IP for all runners (allowlist this on GitLab)"
  value       = aws_eip.nat.public_ip
}

output "private_subnet_id" {
  description = "Private subnet ID where runners operate"
  value       = aws_subnet.private.id
}

output "cache_bucket_arm64" {
  description = "S3 cache bucket for arm64 runners"
  value       = module.gitlab_runner_arm64.runner_cache_bucket_name
}
