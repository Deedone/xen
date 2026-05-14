# Root terragrunt config
# Auto-creates S3 backend + DynamoDB lock table

remote_state {
  backend = "s3"
  generate = {
    path      = "backend.tf"
    if_exists = "overwrite_terragrunt"
  }
  config = {
    bucket         = "xen-ci-terraform-state"
    key            = "${path_relative_to_include()}/terraform.tfstate"
    region         = "eu-central-1"
    encrypt        = true
    dynamodb_table = "terraform-locks"

    # Terragrunt auto-creates these if they don't exist
    s3_bucket_tags = {
      Project   = "xen-ci"
      ManagedBy = "terragrunt"
    }
    dynamodb_table_tags = {
      Project   = "xen-ci"
      ManagedBy = "terragrunt"
    }
  }
}

# Common inputs for all modules
inputs = {
  aws_region  = "eu-central-1"
  environment = "ci"
}
