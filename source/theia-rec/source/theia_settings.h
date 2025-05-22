
#ifndef THEIA_SETTINGS_H
#define THEIA_SETTINGS_H

#define DOWNSAMPLE_CHIRP 8
#define CHIRP_OFFSET 0

#define Q_DEPTH 440

// MTU on psoc6 is 1460  Save room for IP/UDP headers
// Must be a multiple of 4 * DOWNSAMPLE_CHIRP 
#define MAX_UDP_XMIT 1408

#endif