/*
 * wav.h
 *
 *  Created on: Jun 8, 2023
 *      Author: klapl
 */

#ifndef INC_WAV_H_
#define INC_WAV_H_

/*
 * Structure of Header and header variable
 */
typedef struct wavfile_header_s
{
    char    ChunkID[4];     /*  4   */
    uint32_t ChunkSize;      /*  4   */
    char    Format[4];      /*  4   */

    char    Subchunk1ID[4]; /*  4   */
    uint32_t Subchunk1Size;  /*  4   */
    uint16_t AudioFormat;    /*  2   */
    uint16_t NumChannels;    /*  2   */
    uint32_t SampleRate;     /*  4   */
    uint32_t ByteRate;       /*  4   */
    uint16_t BlockAlign;     /*  2   */
    uint16_t BitsPerSample;  /*  2   */

    char    Subchunk2ID[4];
    uint32_t Subchunk2Size;
};

/*Standard values for CD-quality audio*/
#define SUBCHUNK1SIZE   (16)
#define AUDIO_FORMAT    (1) /*For PCM*/
#define NUM_CHANNELS    (1)
#define SAMPLE_RATE     (192000)

#define BITS_PER_SAMPLE (16)

#define BYTE_RATE       (SAMPLE_RATE * NUM_CHANNELS * BITS_PER_SAMPLE / 8)
#define BLOCK_ALIGN     (NUM_CHANNELS * BITS_PER_SAMPLE / 8)


#endif /* INC_WAV_H_ */
