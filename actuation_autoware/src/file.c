// Copyright (c) 2024-2025, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>
#include <errno.h>
#include <zephyr/fs/fs.h>

struct zephyr_file {
    struct fs_file_t file;
    bool in_use;
};

// Maximum number of files that can be open simultaneously
#define MAX_OPEN_FILES 8
static struct zephyr_file files[MAX_OPEN_FILES];

FILE* fopen(const char* filename, const char* mode)
{
  int flags = 0;
  struct zephyr_file *f = NULL;
  int i;

  // Convert mode string to flags
  if (mode[0] == 'r') {
      flags = FS_O_READ;
      if (mode[1] == '+') {
          flags |= FS_O_WRITE;
      }
  } else if (mode[0] == 'w') {
      flags = FS_O_WRITE | FS_O_CREATE;
      if (mode[1] == '+') {
          flags |= FS_O_READ;
      }
  } else if (mode[0] == 'a') {
      flags = FS_O_WRITE | FS_O_CREATE | FS_O_APPEND;
      if (mode[1] == '+') {
          flags |= FS_O_READ;
      }
  } else {
      return NULL;
  }

  // Find a free file slot
  for (i = 0; i < MAX_OPEN_FILES; i++) {
      if (!files[i].in_use) {
          f = &files[i];
          break;
      }
  }

  if (f == NULL) {
      return NULL;  // No free file slots
  }

  int ret = fs_open(&f->file, filename, flags);
  if (ret != 0) {
      return NULL;
  }

  f->in_use = true;
  return (FILE*)f;
}

int fclose(FILE* stream)
{
  struct zephyr_file *f = (struct zephyr_file *)stream;
  
  if (f == NULL || !f->in_use) {
      return EOF;
  }

  int ret = fs_close(&f->file);
  if (ret == 0) {
      f->in_use = false;
      return 0;
  }
  return EOF;
}

size_t fread(void* ptr, size_t size, size_t count, FILE* stream)
{
  struct zephyr_file *f = (struct zephyr_file *)stream;
  
  if (f == NULL || !f->in_use || ptr == NULL) {
      return 0;
  }

  ssize_t bytes_read = fs_read(&f->file, ptr, size * count);
  if (bytes_read < 0) {
      return 0;
  }
  
  return bytes_read / size;
}

size_t fwrite(const void* ptr, size_t size, size_t count, FILE* stream)
{
  struct zephyr_file *f = (struct zephyr_file *)stream;
  
  if (f == NULL || !f->in_use || ptr == NULL) {
      return 0;
  }

  ssize_t bytes_written = fs_write(&f->file, ptr, size * count);
  if (bytes_written < 0) {
      return 0;
  }
  
  return bytes_written / size;
}

int fseek(FILE* stream, long offset, int whence)
{
  struct zephyr_file *f = (struct zephyr_file *)stream;
  
  if (f == NULL || !f->in_use) {
      return -1;
  }

  int fs_whence;
  switch (whence) {
      case SEEK_SET:
          fs_whence = FS_SEEK_SET;
          break;
      case SEEK_CUR:
          fs_whence = FS_SEEK_CUR;
          break;
      case SEEK_END:
          fs_whence = FS_SEEK_END;
          break;
      default:
          return -1;
  }

  int ret = fs_seek(&f->file, offset, fs_whence);
  return (ret == 0) ? 0 : -1;
}


