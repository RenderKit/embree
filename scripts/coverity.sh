#!\bin\bash

module load cmake/3.25.3 &&
export NAS_LINUX=$STORAGE_PATH/packages/apps
cmake --build build --config Release --target build