# Six Can Contest Runner

This package contains maps, configurations, and python apps related to running
a robot in the DPRG six-can competition.

## Current status

The master branch is what was used to run the 2025 competition.

It runs the competition, but the goal response to the first scripted_bot_driver request
after a move using the BasicNavigatorChild gets delayed by 10 - 90 seconds. The result is
it takes 15 minutes to clear the arena, which exceeds the 5-minute limit.

The aider*.md files may not be completely up to date, but are close.