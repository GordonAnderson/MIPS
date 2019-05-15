#ifndef Dialog_h
#define Dialog_h

extern char *DIlist;
extern char *DILlist;

extern char *DOlist;
extern char *DOLlist;


enum DialogTypes
{
  D_MENU,
  D_DIALOG,
  D_TITLE,                      // Display only the title
  D_INT,
  D_INT8,
  D_UINT8,
  D_FLOAT,
  D_BINARY8,
  D_YESNO,
  D_ONOFF,
  D_OPENCLOSE,
  D_FWDREV,
  D_FUNCTION,
  D_LIST,                       // Select from a list of options
  D_PAGE,                       // Changes the display entry pointer and refreshes the dialog
  D_OFF,                        // This will turn an entry off, it will be ignored
  D_STRING,                     // Displays and edits a string. 
  D_DI,                         // Select the Digital input from the list of options, 0 = NA
  D_DILEVEL,
  D_DO,                         // Select the digital output from the list o options, 0 = NA
  D_DOLEVEL
};

typedef struct
{
  const char  *Name PROGMEM;    // Dialog box entry text
  int8_t      X,Y;              // Dialog box entry location
  DialogTypes Type;             // Entry type
  float       Min;
  float       Max;
  float       StepSize;
  int8_t      Xpos;             // X offset for entry location for data display
  bool        NoEdit;           // Flag set to true if field is not editable
  char        *fmt;             // Printf format string for display
  void        *Value;           // Pointer to value data as appropriate
  void (*PreFunction)(void);    // Pointer to function called when entry is selected
  void (*PostFunction)(void);   // Pointer to function called after value entry
}DialogBoxEntry;

typedef struct
{
  Window          w;
  MenuState       State;          // State of DialogBox 
  int8_t          Selected;       // Defines the selected enrty, -1 if not selection
  int8_t          LastUpdated;    //
  bool            Changed;        // Set to true when the user has changed a value in the dialog
  DialogBoxEntry  *Entry;         // Array of Dialog box entries, terminated with NULL
}DialogBox;

extern DialogBox *ActiveDialog;

// Prototypes
void DialogBoxProcessChange(DialogBox *d, int8_t change);
void DialogButtonPress(DialogBox *d);
void DialogValueAdjust(DialogBox *d,int8_t change);
void DialogBoxAdjust(DialogBox *d,int8_t change);
void DisplayDialogEntry(Window *w, DialogBoxEntry *de, bool HighLight);
void UpdateNoEditDialogEntries(Window *w, DialogBoxEntry *de);
void UpdateNoEditDialogEntries(DialogBox *d);
void DisplayAllDialogEntries(DialogBox *d);
void DisplayDialogEntryNames(Window *w, DialogBoxEntry *de, bool HighLight);
void DisplayAllDialogEntryNames(DialogBox *d);
void DialogBoxDisplay(DialogBox *d);
DialogBoxEntry *GetDialogEntries(DialogBoxEntry *de, char *rname);
void RefreshAllDialogEntries(DialogBox *d);
void PrintDialog(DialogBox *d, int X, int Y, char *text);
bool RangeTest(DialogBoxEntry *des, char *EntryName, float fval);

#endif
