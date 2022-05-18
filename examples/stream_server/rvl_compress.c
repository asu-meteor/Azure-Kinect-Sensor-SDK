

int *buffer, *pBuffer, word, nibblesWritten;

void EncodeVLE(int value)
{
    do
    {
        int nibble = value & 0x7; // lower 3 bits
        if (value >>= 3)
            nibble |= 0x8;
        word <<= 4;
        word |= nibble;
        if (++nibblesWritten == 8)
        {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

int DecodeVLE()
{
    unsigned int nibble;
    int value = 0, bits = 29;
    do
    {
        if (!nibblesWritten)
        {
            word = *pBuffer++; //   load word
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);
    return value;
}

int CompressRVL(short *input, char *output, int numPixels)
{
    // size_t passed_array_size = sizeof(*roi) / sizeof(Roi);
    buffer = pBuffer = (int *)output;
    nibblesWritten = 0;
    short *end = input + numPixels;
    short previous = 0;

    while (input != end)
    {
        int zeros = 0, nonzeros = 0;
        for (; (input != end) && !*input; input++, zeros++)
            ;
        EncodeVLE(zeros); // number of zeros
        for (short *p = input; (p != end) && *p++; nonzeros++)
            ;
        EncodeVLE(nonzeros); // number of nonzeros
        for (int i = 0; i < nonzeros; i++)
        {
            short current = *input++;
            int delta = current - previous;
            int positive = (delta << 1) ^ (delta >> 31);
            EncodeVLE(positive); // nonzero value
            previous = current;
        }
    }
    // printf("Num Zeros %d, Num Ones %d\n", totalZeros, totalOnes);
    if (nibblesWritten)
    { // last few values
        *pBuffer++ = word << 4 * (8 - nibblesWritten);
    }
    int result = (int)((char *)pBuffer - (char *)buffer); // num bytes
    return result;
}

void DecompressRVL(char *input, short *output, int numPixels)
{
    buffer = pBuffer = (int *)input;
    nibblesWritten = 0;
    short current, previous = 0;
    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode)
    {
        int zeros = DecodeVLE(); // number of zeros
        numPixelsToDecode -= zeros;
        for (; zeros; zeros--)
            *output++ = 0;
        int nonzeros = DecodeVLE(); // number of nonzeros
        numPixelsToDecode -= nonzeros;
        for (; nonzeros; nonzeros--)
        {
            int positive = DecodeVLE(); // nonzero value
            int delta = (positive >> 1) ^ -(positive & 1);
            current = previous + delta;
            *output++ = current;
            previous = current;
        }
    }
}
