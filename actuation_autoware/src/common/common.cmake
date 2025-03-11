
# Add source files
list(APPEND APP_SOURCES
  src/common/src/dds_impl/src/dds_impl.cpp
  src/common/src/node/src/node.cpp
)

# Add include directories
list(APPEND APP_INCLUDE_DIRS
  src/common/src/dds_impl/include
  src/common/src/node/include
  src/common/src/message/include
)
