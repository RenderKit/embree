#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
cd $DIR/../..

IFS=$'\n'
filter='opt/local/include/\|opt/local/lib/\|opt/local/share/man/\|Applications/Embree@EMBREE_VERSION_MAJOR@'
FILES+=($(pkgutil --files com.intel.embree-@EMBREE_VERSION@.examples | grep -e $filter | tail -r))
FILES+=($(pkgutil --files com.intel.embree-@EMBREE_VERSION@.lib      | grep -e $filter | tail -r))
FILES+=($(pkgutil --files com.intel.embree-@EMBREE_VERSION@.devel    | grep -e $filter | tail -r))
unset IFS

# exit if no files found
if [ ${#FILES[@]} -eq 0 ]; then
  printf "Embree @EMBREE_VERSION@ not installed!\n"
  exit
fi

# first print all files that would get removed
echo Uninstalling Embree @EMBREE_VERSION@ will remove the following files:
PWD=`pwd`
if [ "$PWD" != "/" ]; then
    PWD=$PWD/
fi
for f in "${FILES[@]}"; do
    printf "  %s%s\n" $PWD "$f"
done

echo "Do you wish to uninstall Embree @EMBREE_VERSION@ by removing these files?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) break;;
        No ) exit;;
    esac
done

# now remove files
echo Uninstalling Embree @EMBREE_VERSION@ ...
for f in "${FILES[@]}"; do
    sudo /bin/rm -vd "$f"
done

sudo /usr/sbin/pkgutil --forget com.intel.embree-@EMBREE_VERSION@.examples
sudo /usr/sbin/pkgutil --forget com.intel.embree-@EMBREE_VERSION@.devel
sudo /usr/sbin/pkgutil --forget com.intel.embree-@EMBREE_VERSION@.lib
