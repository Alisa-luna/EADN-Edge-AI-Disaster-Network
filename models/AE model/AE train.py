import torch
import numpy as np
import torch.nn as nn

class TinyConvAE(nn.Module):
    def __init__(self):
        super().__init__()
        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv1d(6, 16, 7, stride=2, padding=3),
            nn.BatchNorm1d(16),
            nn.ReLU(),
            nn.Conv1d(16, 32, 5, stride=2, padding=2),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.Conv1d(32, 64, 5, stride=2, padding=2),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1)
        )
        self.fc_enc = nn.Linear(64, 16)
        self.fc_dec = nn.Linear(16, 64 * 25)
        self.decoder = nn.Sequential(
            nn.ConvTranspose1d(64, 32, 5, stride=2, padding=2, output_padding=1),
            nn.ReLU(),
            nn.ConvTranspose1d(32, 16, 5, stride=2, padding=2, output_padding=1),
            nn.ReLU(),
            nn.ConvTranspose1d(16, 6, 7, stride=2, padding=3, output_padding=1)
        )

    def forward(self, x):
        z = self.encoder(x)  # (B, 64, 1)
        z = z.view(z.size(0), -1)  # (B, 64)
        latent = self.fc_enc(z)  # (B, 16)
        dec = self.fc_dec(latent)  # (B, 64*25)
        dec = dec.view(-1, 64, 25)  # (B, 64, 25)
        out = self.decoder(dec)  # (B, 6, 200)
        return out, latent


def export():
    model = TinyConvAE()
    model.load_state_dict(torch.load('tiny_ae.pth', map_location='cpu'))
    model.eval()

    # 权重映射表：PyTorch名称 → C数组名
    name_map = {
        'encoder.0.weight': 'ae_enc_conv1_w',
        'encoder.0.bias': 'ae_enc_conv1_b',
        'encoder.1.weight': 'ae_enc_bn1_w',
        'encoder.1.bias': 'ae_enc_bn1_b',
        'encoder.1.running_mean': 'ae_enc_bn1_rm',
        'encoder.1.running_var': 'ae_enc_bn1_rv',
        'encoder.3.weight': 'ae_enc_conv2_w',
        'encoder.3.bias': 'ae_enc_conv2_b',
        'encoder.4.weight': 'ae_enc_bn2_w',
        'encoder.4.bias': 'ae_enc_bn2_b',
        'encoder.4.running_mean': 'ae_enc_bn2_rm',
        'encoder.4.running_var': 'ae_enc_bn2_rv',
        'encoder.6.weight': 'ae_enc_conv3_w',
        'encoder.6.bias': 'ae_enc_conv3_b',
        'fc_enc.weight': 'ae_fc_enc_w',
        'fc_enc.bias': 'ae_fc_enc_b',
        'fc_dec.weight': 'ae_fc_dec_w',
        'fc_dec.bias': 'ae_fc_dec_b',
        'decoder.0.weight': 'ae_dec_ct1_w',
        'decoder.0.bias': 'ae_dec_ct1_b',
        'decoder.2.weight': 'ae_dec_ct2_w',
        'decoder.2.bias': 'ae_dec_ct2_b',
        'decoder.4.weight': 'ae_dec_ct3_w',
        'decoder.4.bias': 'ae_dec_ct3_b',
    }

    with open('ae_weights.h', 'w') as f:
        f.write('#ifndef AE_WEIGHTS_H\n#define AE_WEIGHTS_H\n\n')
        f.write('// Auto-generated AE weights\n\n')

        state = model.state_dict()
        for py_name, c_name in name_map.items():
            if py_name in state:
                data = state[py_name].cpu().numpy().flatten()
                f.write(f'static const float {c_name}[{len(data)}] = {{\n  ')
                for i in range(0, len(data), 6):
                    line = ', '.join(f'{x:.8f}f' for x in data[i:i + 6])
                    f.write(line)
                    if i + 6 < len(data):
                        f.write(',\n  ')
                f.write('\n};\n\n')

        f.write('#endif // AE_WEIGHTS_H\n')

    print("权重已导出: ae_weights.h")


if __name__ == '__main__':
    export()