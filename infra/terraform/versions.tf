terraform {
  required_version = ">= 1.5"

  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = ">= 5.76.0"
    }
  }

  # Backend managed by Terragrunt — do not configure here
  # Run: cd infra/terraform && terragrunt init
}

provider "aws" {
  region = var.aws_region
}
