# Child terragrunt config for gitlab-runner module

include "root" {
  path = find_in_parent_folders()
}

terraform {
  source = "."

  extra_arguments "init_reconfigure" {
    commands  = ["init"]
    arguments = ["-reconfigure", "-upgrade"]
  }
}

inputs = {
  gitlab_url = "https://gitpct.epam.com"
}
