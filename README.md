# Micromouse

## Library Setup
To setup the libraries on your machine. You will need to symlink your local `/2017,2018/libraries` to your Arduino libraries folder. This will mean that everytime you pull down updated libraries from Github, they will also be updated for Arduino without any nastiness on your part.

(Instructions for OSX/Mac, similar for Linux)
1. `cd` into your local copy of the repository and `cd` into `/2017,2017/libraries`.
2. Use `pwd` to get the current directory information and save this for later.
3. `cd` to where your Arduino libraries are located. This should be something like `/Users/your_username/Documents/Arduino/Libraries`. Again use `pwd` to save the full directory path.
4. Run `ln -s source destination` and replace `source` with the first directory path and replace `destination` with the second directory path.

Use Finder to verify that the libraries are now in your Arduino folder. This should be a one-time process.
