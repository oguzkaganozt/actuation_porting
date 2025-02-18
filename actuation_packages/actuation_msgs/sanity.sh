#! /bin/bash

# Remove all lines that contain @verbatim
sed -i '/@verbatim/d' log.txt

# Remove all lines that contain @default
sed -i '/@default/d' log.txt

# Remove all lines that contain No default extensibility
sed -i '/No default extensibility/d' log.txt

# Remove all lines that contain Building C object
sed -i '/Building C object/d' log.txt