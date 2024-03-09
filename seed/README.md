# Orange-Seed
This is an experimental repository to 
1. host shared code such as util functions or Odometry across the two bots' code bases
2. use as a potential template creation repository in the future, for easier creation of a new project, without manually copying lots of scripts, as the projects get bigger

Inspired by [RIT Core](https://github.com/RIT-VEX-U/Core)

## How to use OrangeSeed (Unix)

1. If you currently don't have a copy of the OrangeSeed repository, you can clone it by running the following command in your V5 project root folder terminal:
```bash
git subtree add --prefix seed https://github.com/SU-Orange-Robotics/OrangeSeed.git main --squash
```
2. Make sure you have permision to execute the scripts under `./seed/scripts `, you can run this in your V5 project root folder terminal (if you want more safety don't use star, but instead list each file):
```bash
chmod +x seed/scripts/*.sh
```
3. run the scripts, for example
```bash
./seed/scripts/pull-from-seed.sh
```

NOTE:
Before pulling, run `./seed/scripts/check-before-pull.sh` to ensure there is something to pull. Otherwise git subtree pull will force creating empty commit histories.
