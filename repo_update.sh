make distclean
git submodule sync
git submodule update --init --recursive
git submodule update --remote --recursive src/lib/ecl
git submodule update --remote --recursive mavlink
