#!/bin/bash

git add -A
read -p 'commit tag: ' commit_tag
git commit -m $commit_tag

git push origin
