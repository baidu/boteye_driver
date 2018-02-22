#!/bin/bash
# Configure the git-hook if it's not set yet
# Silent if the git-hook is configured correctly.
MASTER_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/../
precommit_hook="${MASTER_DIR}/.git/hooks/pre-commit"
if [[ ! -f "${precommit_hook}" ]]; then
  echo "precommit_hook doesn't exist"
  ln -s ${MASTER_DIR}/utils/git_hooks/pre-commit ${precommit_hook}
  echo "precommit_hook symlink is created"
else
  echo "precommit_hook is already enabled"
fi

