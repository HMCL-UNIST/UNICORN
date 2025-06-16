import torch
import torch.nn as nn

class SimpleNN(nn.Module):
    def __init__(self, weight_decay=0.01):
        super(SimpleNN, self).__init__()
        self.fc1 = nn.Linear(5, 8)  # Input layer
        self.relu = nn.LeakyReLU()
        self.fc2 = nn.Linear(8, 8) # First hidden layer
        self.fc3 = nn.Linear(8, 2) # Output layer

        # Apply Xavier/Glorot initialization
        self._initialize_weights()

        self.weight_decay = weight_decay

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        return self.fc3(x)

    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, mean=0.0, std=0.01)
                nn.init.constant_(m.bias, 0.0)

    def l2_regularization_loss(self):
        l2_reg = torch.tensor(0.0, dtype=torch.float32, device=self.fc1.weight.device)
        for param in self.parameters():
            l2_reg += torch.norm(param)
        return self.weight_decay * l2_reg
