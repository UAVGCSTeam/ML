"""
Use this file to check the status of the CUDA and torch installations.
"""

import torch
print('Torch CUDA is available:\t', end='')
print(torch.cuda.is_available())  # Should return True
print('Device count:\t\t\t', end='')
print(torch.cuda.device_count())  # Should return 1
print('Torch CUDA device name:\t\t', end='')
print(torch.cuda.get_device_name(0))  # Should print Orin
