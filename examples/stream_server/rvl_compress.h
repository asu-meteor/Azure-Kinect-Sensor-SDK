#ifndef RVL_COMPRESS_H
#define RVL_COMPRESS_H

void EncodeVLE(int value);
void DecodeVLE();
void CompressRVL(short *input, char *output, int numPixels);
void DecompressRVL(char *input, short *output, int numPixels);
#endif
