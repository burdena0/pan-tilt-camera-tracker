import torch
from torch import nn
from torch.nn import functional as F
def block(a, b):
    return nn.Sequential(nn.Conv2d(a,b,3,padding=1),nn.ReLU(),nn.Conv2d(b,b,3,padding=1),nn.ReLU())
class SegmentationNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.enc1,self.enc2,self.middle = block(3,8),block(8,16),block(16,24)
        self.dec2,self.dec1,self.head = block(40,16),block(24,8),nn.Conv2d(8,3,1)
    def forward(self,x):
        a=self.enc1(x); b=self.enc2(F.max_pool2d(a,2)); c=self.middle(F.max_pool2d(b,2))
        d=self.dec2(torch.cat([F.interpolate(c,scale_factor=2,mode='nearest'),b],1))
        return self.head(self.dec1(torch.cat([F.interpolate(d,scale_factor=2,mode='nearest'),a],1)))
