import win32file


BLOCK_SIZE = 512        # vast majority of sd cards use 512-byte blocks by default


drive_number = 1  # Replace 2 with the appropriate number for your SD card
file_name = f"\\\\.\\PhysicalDrive{drive_number}"

if __name__ == '__main__':
    handle = win32file.CreateFile(file_name, win32file.GENERIC_READ, win32file.FILE_SHARE_READ | win32file.FILE_SHARE_WRITE, None, win32file.OPEN_EXISTING, 0, None)
    # read data at start of each block until empty block is reached
    while True:

        # do everything 1 block at a time
        bytes_data = win32file.ReadFile(handle, BLOCK_SIZE)

        # blocks consist of possible data followed by 0x00 filling the rest of the block
        data = block
        for i in range(len(block)):
            if block[i] == '\0': # same as 0x00
                data = data[:i]
                break
            
        # if block started with 0x00, this block contains no data
        if len(data) == 0:
            break
            
        # you'll probably want to use bash to pipe me to a file
        print(data, end='')
# Close the handle
win32file.CloseHandle(handle)