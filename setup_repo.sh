#!/usr/bin/env bash
# setup_repo.sh — initialise the git repository and push to GitHub
#
# Usage (Ubuntu / Git Bash on Windows):
#   bash setup_repo.sh https://github.com/YOUR_USERNAME/ros-mesh-preprocessor.git

set -e

REMOTE_URL=${1:-""}

echo "Initialising git repository …"
git init
git add .
git commit -m "feat: initial commit — ros-mesh-preprocessor v0.1.0"

if [ -n "$REMOTE_URL" ]; then
  git remote add origin "$REMOTE_URL"
  git branch -M main
  git push -u origin main
  echo "Pushed to $REMOTE_URL"
else
  echo ""
  echo "No remote URL provided. To push to GitHub:"
  echo "  git remote add origin https://github.com/YOUR_USERNAME/ros-mesh-preprocessor.git"
  echo "  git branch -M main"
  echo "  git push -u origin main"
fi
