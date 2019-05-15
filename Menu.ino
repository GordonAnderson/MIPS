//
// Menu.c
//
//
#include "Menu.h"

Menu *ActiveMenu = NULL;

uint16_t x;
uint16_t y;
uint16_t w;
uint16_t h;
uint8_t ch;
uint8_t cw;
uint8_t cHeight;
uint8_t cWidth;
uint8_t offset;

bool PopupDismissed = true;

void CalculatePositionVariables(Window *wn)
{
  x = wn->Xorgin + 4;  // Adjust for border
  y = wn->Yorgin + 4;
  w = wn->Width - 8;
  h = wn->Height - 8;
  // Adjust the option size area if a title is present
  if (wn->Title != NULL)
  {
    y += wn->Tsize * (C_HEIGHT + 1);
    h -= wn->Tsize * (C_HEIGHT + 1);
  }
  // Determine the character heights and widths
  cHeight = wn->Tsize * (C_HEIGHT + 1);
  cWidth = wn->Tsize * (C_WIDTH + 1);
  // Determine the display area size in characters
  ch = h / cHeight;
  cw = (w - y) / cWidth;
  if (wn->MaxLines < ch) ch = wn->MaxLines;
}

void p(char *fmt, ... )
{
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  tft.print(tmp);
}

// This function is called when an encoder position chnage is detected.
// This function will check the state and process the change.
void MenuProcessChange(Menu *m, int8_t change)
{
  if (m->State == M_SCROLLING) MenuAdjust(m, change);
  if (m->State == M_ENTRYSELECTED) MenuValueAdjust(m, change);
}

// This function is called when the menu encoder button press is detected.
// The function will check the state and process the button press.
void MenuButtonPress(Menu *m)
{
  void (*function)(void);

  // If state is scrolling then see if this is an entry that we can take action with, if so
  // do it, else ignore the button
  if (m->State == M_SCROLLING)
  {
    if (m->Entry[m->Selected].PreFunction != NULL) m->Entry[m->Selected].PreFunction();
    // Select value and change state to entry selected
    switch (m->Entry[m->Selected].Type)
    {
      case M_MENU:
        if (m->Entry[m->Selected].Value != NULL)
        {
          MenuDisplay((Menu *)m->Entry[m->Selected].Value);
          return;
        }
        break;
      case M_DIALOG:
        if (m->Entry[m->Selected].Value != NULL)
        {
          DialogBoxDisplay((DialogBox *)m->Entry[m->Selected].Value);
          if (m->Entry[m->Selected].PostFunction != NULL) m->Entry[m->Selected].PostFunction();
          return;
        }
        break;
      case M_BINARY8_RO:
        if (m->Entry[m->Selected].Value != NULL) DisplayEntry(m, m->Selected, false);
        break;
      case M_BINARY8:
      case M_YESNO:
      case M_INT:
        if (m->Entry[m->Selected].Value != NULL)
        {
          DisplayEntry(m, m->Selected, true);
          m->State = M_ENTRYSELECTED;
        }
        break;
      case M_FUNCTION:
        break;
      default:
        break;
    }
  }
  else if (m->State == M_ENTRYSELECTED)
  {
    // Deselect call post function and return to scrolling
    DisplayEntry(m, m->Selected, false);
    if (m->Entry[m->Selected].PostFunction != NULL) m->Entry[m->Selected].PostFunction();
    m->State = M_SCROLLING;
  }
}

void MenuAdjust(Menu *m, int8_t change)
{
  // First determine the number of menu etries.
  int i = 0;
  while (m->Entry[i].Name != NULL) i++;
  DisplaySelected(m, false);
  m->Selected += change;
  if (m->Selected < 0) m->Selected = i - 1;
  else if (m->Selected > i - 1) m->Selected = 0;
  DisplaySelected(m, true);
}

void MenuValueAdjust(Menu *m, int8_t change)
{
  if (m->Entry[m->Selected].Value == NULL) return;
  switch (m->Entry[m->Selected].Type)
  {
    case M_INT:
      *(int *)m->Entry[m->Selected].Value += change;
      if (*(int *)m->Entry[m->Selected].Value > m->Entry[m->Selected].Max) *(int *)m->Entry[m->Selected].Value = m->Entry[m->Selected].Max;
      if (*(int *)m->Entry[m->Selected].Value < m->Entry[m->Selected].Min) *(int *)m->Entry[m->Selected].Value = m->Entry[m->Selected].Min;
      break;
    case M_BINARY8:
      *(int8_t *)m->Entry[m->Selected].Value += change;
      if (*(int8_t *)m->Entry[m->Selected].Value > m->Entry[m->Selected].Max) *(int8_t *)m->Entry[m->Selected].Value = m->Entry[m->Selected].Max;
      if (*(int8_t *)m->Entry[m->Selected].Value < m->Entry[m->Selected].Min) *(int8_t *)m->Entry[m->Selected].Value = m->Entry[m->Selected].Min;
      break;
    case M_YESNO:
      *(bool *)m->Entry[m->Selected].Value += change;
      if (*(bool *)m->Entry[m->Selected].Value > m->Entry[m->Selected].Max) *(bool *)m->Entry[m->Selected].Value = m->Entry[m->Selected].Max;
      if (*(bool *)m->Entry[m->Selected].Value < m->Entry[m->Selected].Min) *(bool *)m->Entry[m->Selected].Value = m->Entry[m->Selected].Min;
      break;
    default:
      break;
  }
  DisplayEntry(m, m->Selected, true);
}

void DisplayEntry(Menu *m, int8_t Selected, bool HighLight)
{
  CalculatePositionVariables(&m->w);
  // Define offset to insure selected item is on the menu
  offset = 0;
  if (m->Selected > (ch - 1)) offset = m->Selected - (ch - 1);
  //
  if (m->Offset != offset) MenuOptions(m);
  offset = m->Offset;
  int i = Selected;
  if (HighLight) tft.setTextColor(m->w.Bcolor, m->w.Fcolor);
  else tft.setTextColor(m->w.Fcolor, m->w.Bcolor);
  switch (m->Entry[i + offset].Type)
  {
    case M_INT:
      if (m->Entry[i + offset].Value != NULL)
      {
        // If here then we have a valid pointer to a interger value.
        // Use the format line to print the result to the display
        // at the selected X offset.
        tft.setCursor(x + m->Entry[i + offset].Xpos * m->w.Tsize * (C_WIDTH + 1), y + i * m->w.Tsize * (C_HEIGHT + 1));
        p(m->Entry[i + offset].fmt, *(int *)m->Entry[i + offset].Value);
      }
      break;
    case M_YESNO:
      if (m->Entry[i + offset].Value != NULL)
      {
        tft.setCursor(x + m->Entry[i + offset].Xpos * m->w.Tsize * (C_WIDTH + 1), y + i * m->w.Tsize * (C_HEIGHT + 1));
        if (*(bool *)m->Entry[i + offset].Value) p("YES");
        else p("NO ");
      }
      break;
    case M_BINARY8_RO:
    case M_BINARY8:
      if (m->Entry[i + offset].Value != NULL)
      {
        tft.setCursor(x + m->Entry[i + offset].Xpos * m->w.Tsize * (C_WIDTH + 1), y + i * m->w.Tsize * (C_HEIGHT + 1));
        for (int j = 7; j >= 0; j--)
        {
          if (*(int8_t *)m->Entry[i + offset].Value & (1 << j)) p("1");
          else p("0");
        }
      }
      break;
    default:
      break;
  }
}

// This function displays the selected entry, if its not displayed on the
// menu then the menu display range will firt be updated.
void DisplaySelected(Menu *m, bool HighLight)
{
  CalculatePositionVariables(&m->w);
  // Define offset to insure selected item is on the menu
  offset = 0;
  if (m->Selected > (ch - 1)) offset = m->Selected - (ch - 1);
  //
  if (m->Offset != offset) MenuOptions(m);
  offset = m->Offset;
  // Highlight the selected option
  tft.setCursor(x, y + (m->Selected - offset) * m->w.Tsize * (C_HEIGHT + 1));
  if (HighLight) tft.setTextColor(m->w.Bcolor, m->w.Fcolor);
  else tft.setTextColor(m->w.Fcolor, m->w.Bcolor);
  p("%*-s", strlen(m->Entry[m->Selected].Name), m->Entry[m->Selected].Name);
}

// This function prints the menu selections and makes sure the selected
// option is on the screen.
void MenuOptions(Menu *m)
{
  CalculatePositionVariables(&m->w);
  // Define offset to insure selected item is on the menu
  offset = 0;
  if (m->Selected > (ch - 1)) offset = m->Selected - (ch - 1);
  //
  m->Offset = offset;
  // Display the options
  for (int i = 0; i < ch; i++)
  {
    if (m->Entry[i + offset].Name == NULL) break;
    tft.setCursor(x, y + i * m->w.Tsize * (C_HEIGHT + 1));
    tft.setTextColor(m->w.Fcolor, m->w.Bcolor);
    p("%*-s", cw, m->Entry[i + offset].Name);
    // Display the data depending on the menu entry selection type
    DisplayEntry(m, i, false);
  }
}

// Sets the cursor text position in the window
void SetWindowTextPos(Window *w, uint8_t X, uint8_t Y)
{
  CalculatePositionVariables(w);
  tft.setCursor(x + X * cWidth, y + Y * cHeight);
}

void DisplayWindow(Window *wn)
{
  uint16_t x = wn->Xorgin;
  uint16_t y = wn->Yorgin;
  uint16_t w = wn->Width;
  uint16_t h = wn->Height;

  // Clear the menu area of screen
  tft.fillRect(x, y, w, h, wn->Bcolor);
  // Print the title and center, if defined
  if (wn->Title != NULL)
  {
    tft.setCursor(x + (w - (strlen(wn->Title) * wn->Tsize * (C_WIDTH + 1))) / 2, y);
    tft.setTextColor(wn->Fcolor, wn->Bcolor);
    tft.setTextSize(wn->Tsize);
    tft.println(wn->Title);
    y += wn->Tsize * (C_HEIGHT + 1);
    h -= wn->Tsize * (C_HEIGHT + 1);
  }
  // Draw the border
  if (wn->Border != B_NONE)
  {
    tft.drawRect(x, y, w, h, wn->Fcolor);
    if (wn->Border == B_DOUBLE) tft.drawRect(x + 2, y + 2, w - 4, h - 4, wn->Fcolor);
  }
}

// This function draws the box for the selection menu and clears
// the menu area and then displays the initial menu with the selected
// option highlighted.
// This is called to initially setup and display a menu.
void MenuDisplay(Menu *m)
{
  if (m == NULL) return;
  DisplayWindow(&m->w);
  // Display all the menu options and highlight the selected option
  MenuOptions(m);
  DisplaySelected(m, true);
  ActiveDialog = NULL;
  ActiveMenu = m;
}

// This function will pop up a message in the middle of the current
// display. The active menu and active dislogbox pointers will
// be saved and then restored when the message is dismissed.
Menu *saveActiveMenu = NULL;
DialogBox *saveActiveDialog = NULL;

// Define message window
Window mw =  {" System Message ",
              ILI9340_BLACK,
              ILI9340_RED,
              2,
              20,75,
              260,50,
              B_DOUBLE,
              1};
              
// Display dismiss Thread
Thread DisplayDismissThread  = Thread();

// Displays a pop up message and it remains until DismissMessage is called
void DisplayMessage(char *message)
{
  DisplayIntensity();
  // Exit there is something to restore, thus popup message is already on screen
  if((saveActiveMenu != NULL) || (saveActiveDialog != NULL)) return;
  // Save the menu and dialogbox pointers then set to null. This
  // will stop screen updates
  if(saveActiveMenu == NULL); saveActiveMenu = ActiveMenu;
  if(saveActiveDialog == NULL) saveActiveDialog = ActiveDialog;
  ActiveMenu = NULL;
  ActiveDialog = NULL;
  // Display the window
  DisplayWindow(&mw);
  // Print the message and center it
  CalculatePositionVariables(&mw);
  tft.setCursor(x + (w - (strlen(message) * mw.Tsize * (C_WIDTH + 1))) / 2, y + 4);
  tft.setTextColor(mw.Fcolor, mw.Bcolor);
  tft.setTextSize(mw.Tsize);
  tft.println(message);
}

// Displays a pop up message and it remains until the user presses the controlknob
void DisplayMessageButtonDismiss(char *message)
{
  // Display the message
  DisplayMessage(message);
  // Set flag that will enable the control knob to dismiss the message
  PopupDismissed = false;
}

// Dismiss this message and kill this thread.
// This thread will be called as soon as its added to the thread list and again
// after the proper delay. Kill it on the second call.
int NumCalls = 0;

void DismissMessageThread(void)
{
  if(NumCalls++ == 0) return;
  DismissMessage();
  control.remove(&DisplayDismissThread);
}

// This function displays a message for a defined length of time in millisec
void DisplayMessage(char *message, int DisplayTime)
{
  // Display the message
  DisplayMessage(message);
  // Schedule a task to dismiss the message after DisplayTime
  // Configure Threads
  DisplayDismissThread.onRun(DismissMessageThread);
  DisplayDismissThread.setInterval(DisplayTime);
  // Add threads to the controller
  control.add(&DisplayDismissThread);
  NumCalls=0;
}

// Restore the pointers and refresh the screen
void DismissMessage(void)
{
  // Exit if nothing to restore
  if((saveActiveMenu == NULL) && (saveActiveDialog == NULL)) return;
  // Restore the pointers
  ActiveMenu = saveActiveMenu;
  ActiveDialog = saveActiveDialog;
  saveActiveMenu = NULL;
  saveActiveDialog = NULL;
  // Clear any button flags
  ButtonPressed = false;
  ButtonRotated = false;
  // Refresh the screen
  MenuDisplay(ActiveMenu);
  DialogBoxDisplay(ActiveDialog);
}

// Restore the pointers and refresh the screen if the button was pressed and we are waiting for the button
// to be pressed
void DismissMessageIfButton(void)
{
  if(PopupDismissed) return;
  PopupDismissed = true;
  DismissMessage();
}
