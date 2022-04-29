#!/bin/bash

SRC_DIR=$1
DST_DIR=$2

SYCL_RUNNERS="\
  vis-ci-cl-00 \
  vis-ci-cl-01 \
  vis-ci-cl-02 \
  vis-ci-cl-03 \
  "

#for RUNNER in $SYCL_RUNNERS
#do
#  echo $RUNNER
#  # copy all deb files in SRC_DIR in folder DST_DIR
#  ssh visuser@$RUNNER.an.intel.com "mkdir -p ${DST_DIR}"
#  scp -rp $SRC_DIR/*.deb visuser@$RUNNER.an.intel.com:$DST_DIR &
#done
#
#wait

# install all deb files
for RUNNER in $SYCL_RUNNERS
do
  ssh -t visuser@$RUNNER.an.intel.com "sudo dpkg -i ${DST_DIR}/*.deb"
done
