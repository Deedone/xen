# Child terragrunt config for gitlab-runner module

include "root" {
  path = find_in_parent_folders()
}

terraform {
  source = "."
}

inputs = {
  gitlab_url = "https://gitpct.epam.com"
}
