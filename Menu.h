#ifndef Menu_h
#define Menu_h

// Characters are 5x7 pixels with a one pixed space to the right and the bottom.
// Charater size is a multiplier on the pixel size.
#define  C_HEIGHT    7
#define  C_WIDTH     5

enum MenuState
{
  M_SCROLLING,
  M_ENTRYSELECTED,
};

enum MenuTypes
{
  M_MENU,
  M_DIALOG,
  M_INT,
  M_BINARY8,     // Display and edit binary value
  M_BINARY8_RO,  // This function will display a binary value but no edit is allowed
  M_YESNO,
  M_FUNCTION
};

enum MenuBorder
{
  B_NONE,
  B_SINGLE,
  B_DOUBLE
};

typedef struct
{
  char        *Name;            // Menu entry text
  MenuTypes   Type;             // Entry type
  int16_t     Min;
  int16_t     Max;
  int8_t      Xpos;             // Position for data display
  char        *fmt;             // Printf format string for display
  void        *Value;           // Pointer to value data as appropriate
  void (*PreFunction)(void);    // Pointer to function called when entry is selected
  void (*PostFunction)(void);   // Pointer to function called after value entry
}MenuEntry;

typedef struct
{
  char        *Title;         // Menu title, NULL if no title wanted
  uint16_t    Bcolor;         // Background color
  uint16_t    Fcolor;         // Forground color
  uint8_t     Tsize;          // Text size for menu
  int16_t     Xorgin,Yorgin;  // Menu location
  int16_t     Width, Height;  // Menu size
  MenuBorder  Border;         // Menu border style  
  uint8_t     MaxLines;       // Number of displayed entries
} Window;

typedef struct
{
  Window      w;              // Defines the details of the menu's window
  MenuState   State;          // State of this menu
  int8_t      Selected;       // Defines the selected enrty, -1 if no selection
  uint8_t     Offset;         // Index of the first displayed menu option, used by menu functions
  MenuEntry   *Entry;         // Array of menu entries, terminated with NULL
}Menu;

extern Menu *ActiveMenu;
extern bool PopupDismissed;

// Function prototypes
void p(char *fmt, ... );
void CalculatePositionVariables(Window *wn);
void MenuProcessChange(Menu *m, int8_t change);
void MenuButtonPress(Menu *m);
void MenuAdjust(Menu *m, int8_t change);
void MenuValueAdjust(Menu *m, int8_t change);
void DisplayEntry(Menu *m, int8_t Selected, bool HighLight);
void DisplaySelected(Menu *m, bool HighLight);
void MenuOptions(Menu *m);
void SetWindowTextPos(Window *w, uint8_t X, uint8_t Y);
void DisplayWindow(Window *wn);
void MenuDisplay(Menu *m);
void DisplayMessage(char *message);
void DismissMessage(void);
void DisplayMessageButtonDismiss(char *message);
void DismissMessageIfButton(void);


#endif
