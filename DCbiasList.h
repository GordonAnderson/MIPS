#ifndef DCBIASSTATE_H_
#define DCBIASSTATE_H_

typedef struct
{
   int8_t    Address;  // SPI address for the DAC
   int8_t    Count;
   uint32_t  *data;   // This is the DMA data buffer, three words per transfer.
                      // DMA does 16 bit transfers to SPI, two transfers for 
                      // each channel update then a transfer to a dummy SPI
                      // chip select to cause strobe.
} ModuleData;

typedef struct DCstate DCstate;

// Linked list of states, a state defines a set of DC bias voltage values.
struct DCstate
{
  char        name[10];
  DCstate     *next;
  ModuleData  md[4];
};

typedef struct
{
  uint32_t       Count;         // Count for this state to apply
  void (*Setup)(void);          // Pointer to function called to setup this time point, prep for the LDAC pulse
  uint32_t       DIOimage;      // Used for trigger generation and contains the SPI word output
  uint8_t        NumStates;     // Number of states in array
  DCstate        **States;      // Pointer to an array of state pointers
} DCsegmentTP;

// Segments define timed behavior. When a segment is triggered or started then after a defined time
// DC voltages states are set.
typedef struct DCsegment DCsegment;

struct DCsegment
{
  char         name[10];
  DCsegment    *next;
  DCsegment    *repeat;
  uint32_t     Length;          // Segment length in counts
  uint8_t      RepeatCount;     // Repeat count
  uint8_t      CurrentCount;
  uint8_t      NumTimePoints;   // Number of time points in this segment
  DCsegmentTP  **TimePoint;     // Pointer to an array of time point pointers
};


void PlaySegments(void);

// Prototypes for segments and there time points
void DefineSegment(void);        // Defines a segment with the following arguments: name, next, repeat count
void AddSegmentTimePoint(void);  // Adds a time point to a segment, arguments: name,count, state1, state 2... (variable number of states)
void AddSegmentTrigger(void);
void RemoveSegment(char *name);  // Removes the named segment
void RemoveSegments(void);       // Removes all segments and frees memory
void ListSegments(void);

// Prototypes for the DCstates
void DefineState(void);
void SetState(char *name);
void ListStateNames(void);
void RemoveState(char *name);
void RemoveStates(void);
void IsState(char *name);

#endif

