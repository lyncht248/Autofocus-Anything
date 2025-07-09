#!/bin/bash

set -e

# List of remaining repos (after HVI_desktop_app)
REPOS=(
  "autofocus-app"
  "autofocus-app-newActuator"
  "VIVID-Autofocus"
)

# Map repo to previous one (for commit messages)
PREV_REPOS=(
  "HVI_desktop_app"
  "autofocus-app"
  "autofocus-app-newActuator"
)

for i in "${!REPOS[@]}"; do
  repo="${REPOS[$i]}"
  prev="${PREV_REPOS[$i]}"
  remote_name="src"

  echo "=== Merging $repo ==="

  git remote add $remote_name "../$repo"
  git fetch $remote_name

  # Get default branch name
  DEFAULT_BRANCH=$(git remote show $remote_name | grep 'HEAD branch' | awk '{print $NF}')

  # Get the first commit date of this repo
  NEXT_FIRST_COMMIT_DATE=$(git --no-pager log $remote_name/$DEFAULT_BRANCH --reverse --format="%aI" | head -n 1)
  TRANSITION_DATE=$(date -u -d "$(echo $NEXT_FIRST_COMMIT_DATE | sed 's/Z//') -1 seconds" +"%Y-%m-%dT%H:%M:%S")

  # Add empty transition commit
  COMMIT_MSG="🔀 Transition: moved from $prev to $repo"
  echo "Adding transition commit before $repo: $COMMIT_MSG at $TRANSITION_DATE"
  GIT_COMMITTER_DATE="$TRANSITION_DATE" git commit --allow-empty -m "$COMMIT_MSG" --date="$TRANSITION_DATE"

  # Merge the repo
  git merge --allow-unrelated-histories "$remote_name/$DEFAULT_BRANCH" -m "Merged $repo"

  # Tag the merge
  tag_name="v$((i+3))-${repo}-final"  # Already did v1 and v2 manually
  git tag "$tag_name"

  git remote remove $remote_name
done

echo "✅ Remaining repos merged successfully."
