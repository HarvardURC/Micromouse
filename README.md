# Micromouse

## Overall Structure

This repo contains both the 2015-2017 code and the 2017-2018 code, in the respectively named folders. In 2017-2018 we had both an experienced and a beginner team. They used different Micromice and have different codebases. The codebase merged into master right now is the experienced team's codebase. To look at the beginner team codebase you can switch to the `beg-mouse` branch and look in the 2017,2018 folder.

## Library Setup
To setup the libraries on your machine you will need to symlink your local `/2017-2018/libraries` to your Arduino libraries folder. This will mean that every time you pull down updated libraries from Github they will also be updated for your local Arduino application without any nastiness on your part. The default location for your local Arduino libraries folder is:
* OSX: `/Users/your_username/Documents/Arduino/Libraries`
* Windows: `My Documents\Arduino\libraries\`

(Instructions for OSX/Mac, similar for Linux)
1. `cd` into your local copy of the repository and `cd` into `/2017-2018/libraries`.
2. Use `pwd` to get the current directory information and save this for later.
3. `cd` to where your Arduino libraries are located. Again use `pwd` to save the full directory path.
4. Run `ln -s source destination` and replace `source` with the first directory path and replace `destination` with the second directory path.

Use Finder to verify that the libraries are now in your Arduino folder. This should be a one-time process.

(Instructions for Windows) TODO: needs to be verified
1. `cd` into your local copy of the repository and `cd` into `/2017-2018/libraries`.
2. Use `cd` (no arguments) to get the current directory information and save this for later.
3. `cd` to where your Arduino libraries are located. Again use `cd` (no arguments) to save the full directory path.
4. Run `mklink /D source destination` and replace `source` with the first directory path and replace `destination` with the second directory path.
* Note: If symlink does not work, simply locally cp (copy) the necessary libraries into your Arduino library folder
