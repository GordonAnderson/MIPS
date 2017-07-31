//
// Dialog
//
// This file contains functions to support display of a dialog box on the display.
//
//
#include "Dialog.h"
#include "Menu.h"

char *DIlist = "NA,Q,R,S,T,U,V,W,X";
char *DILlist = "NA,LOW,HIGH,BOTH,NEG,POS";
// Arduino system constants
//  Low = 0
//  High = 1
//  Change = 2
//  Falling = 3
//  Rising = 4
// The trigger/level list need to be in this order

char *DOlist = "NA,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P";
char *DOLlist = "NA,LOW,HIGH";

DialogBox *ActiveDialog = NULL;

// This function is called when an encoder position change is detected.
// This function will check the state and process the change.
void DialogBoxProcessChange(DialogBox *d, int8_t change)
{
  if (change == 0) return;
  if (d->State == M_SCROLLING)
  {
    if (change > 1) change = 1;
    if (change < -1) change = -1;
    DialogBoxAdjust(d, change);
  }
  if (d->State == M_ENTRYSELECTED) DialogValueAdjust(d, change);
}

// This function is called when the encoder button press is detected.
// The function will check the state and process the button press.
void DialogButtonPress(DialogBox *d)
{
  void (*function)(void);

  // If state is scrolling then see if this is an entry that we can take action with, if so
  // do it, else ignore the button
  if (d->State == M_SCROLLING)
  {
    if (d->Entry[d->Selected].Type == D_FUNCTION)
    {
      if (d->Entry[d->Selected].PreFunction != NULL) d->Entry[d->Selected].PreFunction();
      return;
    }
    if (d->Entry[d->Selected].PreFunction != NULL) d->Entry[d->Selected].PreFunction();
    // Select value and change state to entry selected
    if (d->Entry[d->Selected].NoEdit == false) switch (d->Entry[d->Selected].Type)
      {
        case D_MENU:
          if (d->Entry[d->Selected].Value != NULL)
          {
            MenuDisplay((Menu *)d->Entry[d->Selected].Value);
            return;
          }
          break;
        case D_DIALOG:
          if (d->Entry[d->Selected].Value != NULL)
          {
            DialogBoxDisplay((DialogBox *)d->Entry[d->Selected].Value);
            return;
          }
          break;
        case D_BINARY8:
        case D_YESNO:
        case D_FWDREV:
        case D_ONOFF:
        case D_OPENCLOSE:
        case D_INT:
        case D_INT8:
        case D_UINT8:
        case D_FLOAT:
        case D_LIST:
        case D_DI:
        case D_DO:
        case D_DILEVEL:
        case D_DOLEVEL:
          if (d->Entry[d->Selected].Value != NULL)
          {
            DisplayDialogEntry(&d->w, &d->Entry[d->Selected], true);
            d->State = M_ENTRYSELECTED;
          }
          break;
        case D_PAGE:
          // Update the entries page and refresh the dialog
          d->Entry = (DialogBoxEntry *)d->Entry[d->Selected].Value;
          d->Selected = 0;
          d->State = M_SCROLLING;
          d->LastUpdated = 0;
          DialogBoxDisplay(d);
          break;
        default:
          break;
      }
  }
  else if (d->State == M_ENTRYSELECTED)
  {
    // Deselect, call post function, and return to scrolling
    // Changed order to: Call post function, deselect, and return to scorlling. This allows post
    // function to change displayed value. GAA July 7, 2015
    if (d->Entry[d->Selected].PostFunction != NULL) d->Entry[d->Selected].PostFunction();
    DisplayDialogEntry(&d->w, &d->Entry[d->Selected], false);
    d->State = M_SCROLLING;
  }
}

// This function is called when the encoder knob rotation is being used to adjust a value.
void DialogValueAdjust(DialogBox *d, int8_t change)
{
  int  i;
  char *str,entry[10];
  static unsigned int lastTime = 0;
  static int multiplier = 1;

  if (change == 0) return;
  if (lastTime == 0) lastTime = millis();
  if ((millis() - lastTime) < 200 / multiplier) multiplier *= 10;
  else multiplier /= 10;
  if ((millis() - lastTime) > 500) multiplier = 1;
  if (d->Entry[d->Selected].Value == NULL) return;
  if (multiplier > 100) multiplier = 100;
  if (multiplier < 1) multiplier = 1;
  lastTime = millis();
  switch (d->Entry[d->Selected].Type)
  {
    case D_INT:
      *(int *)d->Entry[d->Selected].Value += change * d->Entry[d->Selected].StepSize * multiplier;
      if (*(int *)d->Entry[d->Selected].Value > d->Entry[d->Selected].Max) *(int *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Max;
      if (*(int *)d->Entry[d->Selected].Value < d->Entry[d->Selected].Min) *(int *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Min;
      break;
    case D_INT8:
      *(int8_t *)d->Entry[d->Selected].Value += change * d->Entry[d->Selected].StepSize * multiplier;
      if (*(int8_t *)d->Entry[d->Selected].Value > d->Entry[d->Selected].Max) *(int8_t *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Max;
      if (*(int8_t *)d->Entry[d->Selected].Value < d->Entry[d->Selected].Min) *(int8_t *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Min;
      break;
    case D_UINT8:
      *(uint8_t *)d->Entry[d->Selected].Value += change * d->Entry[d->Selected].StepSize * multiplier;
      if (*(uint8_t *)d->Entry[d->Selected].Value > d->Entry[d->Selected].Max) *(uint8_t *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Max;
      if (*(uint8_t *)d->Entry[d->Selected].Value < d->Entry[d->Selected].Min) *(uint8_t *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Min;
      break;
    case D_FLOAT:
      *(float *)d->Entry[d->Selected].Value += change * d->Entry[d->Selected].StepSize * multiplier;
      if (*(float *)d->Entry[d->Selected].Value > d->Entry[d->Selected].Max) *(float *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Max;
      if (*(float *)d->Entry[d->Selected].Value < d->Entry[d->Selected].Min) *(float *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Min;
      break;
    case D_BINARY8:
      *(uint8_t *)d->Entry[d->Selected].Value += change * d->Entry[d->Selected].StepSize;
      if (*(uint8_t *)d->Entry[d->Selected].Value > d->Entry[d->Selected].Max) *(uint8_t *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Max;
      if (*(uint8_t *)d->Entry[d->Selected].Value < d->Entry[d->Selected].Min) *(uint8_t *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Min;
      break;
    case D_ONOFF:
    case D_OPENCLOSE:
    case D_YESNO:
    case D_FWDREV:
      *(bool *)d->Entry[d->Selected].Value += change * d->Entry[d->Selected].StepSize;
      if (*(bool *)d->Entry[d->Selected].Value > d->Entry[d->Selected].Max) *(bool *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Max;
      if (*(bool *)d->Entry[d->Selected].Value < d->Entry[d->Selected].Min) *(bool *)d->Entry[d->Selected].Value = d->Entry[d->Selected].Min;
      break;
    case D_LIST:
      // Find current entry in list then adjust to next entry. If this entry is not found then start with
      // first entry.
      d->Entry[d->Selected].Min = 1;
      d->Entry[d->Selected].Max = NumListEntries(d->Entry[d->Selected].fmt);
      i = FindInList(d->Entry[d->Selected].fmt, (char *)d->Entry[d->Selected].Value);
      if (i == -1) i = 1;
      else
      {
        i += change;
        if (i > d->Entry[d->Selected].Max) i = d->Entry[d->Selected].Max;
        if (i < d->Entry[d->Selected].Min) i = d->Entry[d->Selected].Min;
      }
      // Write result into value and we are done
      sprintf((char *)d->Entry[d->Selected].Value, "%s", GetEntry(d->Entry[d->Selected].fmt, i));
      break;
    case D_DI:
      entry[0] = *(char *)d->Entry[d->Selected].Value;
      entry[1] = 0;
      i = FindInList(DIlist,entry);
      if (i < 1) i = 1;
      i += change;
      if (i > NumListEntries(DIlist)) i = NumListEntries(DIlist);
      if (i < 1) i = 1;
      str = GetEntry(DIlist,i);
      *(char *)d->Entry[d->Selected].Value = str[0];
      if(i==1) *(char *)d->Entry[d->Selected].Value = 0;
      break;
    case D_DO:
      entry[0] = *(char *)d->Entry[d->Selected].Value;
      entry[1] = 0;
      i = FindInList(DOlist,entry);
      if (i < 1) i = 1;
      i += change;
      if (i > NumListEntries(DOlist)) i = NumListEntries(DOlist);
      if (i < 1) i = 1;
      str = GetEntry(DOlist,i);
      *(char *)d->Entry[d->Selected].Value = str[0];
      if(i==1) *(char *)d->Entry[d->Selected].Value = 0;
      break;
    case D_DILEVEL:
      i = (int8_t) *(int8_t *)d->Entry[d->Selected].Value + 2 + change;
      if (i > NumListEntries(DILlist)) i = NumListEntries(DILlist);
      if (i < 1) i = 1;
      *(int8_t *)d->Entry[d->Selected].Value = i - 2;
      break;
    case D_DOLEVEL:
      i = (int8_t) *(int8_t *)d->Entry[d->Selected].Value + 2 + change;
      if (i > NumListEntries(DOLlist)) i = NumListEntries(DOLlist);
      if (i < 1) i = 1;
      *(int8_t *)d->Entry[d->Selected].Value = i - 2;
      break;
    default:
      break;
  }
  d->Changed = true;
  DisplayDialogEntry(&d->w, &d->Entry[d->Selected], true);
}

void DialogBoxAdjust(DialogBox *d, int8_t change)
{
  // First determine the number of dialog box etries.
  int i = 0;
  while (d->Entry[i].Name != NULL) i++;
  DisplayDialogEntryNames(&d->w, &d->Entry[d->Selected], false);
  while (1)
  {
    d->Selected += change;
    if (d->Selected < 0) d->Selected = i - 1;
    else if (d->Selected > i - 1) d->Selected = 0;
    if ((d->Entry[d->Selected].Type == D_TITLE) || (d->Entry[d->Selected].Type == D_OFF) || (strlen(d->Entry[d->Selected].Name) == 0))
    {
      if (change > 1) change = 1;
      if (change < -1) change = -1;
      continue;
    }
    DisplayDialogEntryNames(&d->w, &d->Entry[d->Selected], true);
    break;
  }
}


// The function displays the selected dialog entry in the window selected.
void DisplayDialogEntry(Window *w, DialogBoxEntry *de, bool HighLight)
{
  int  i;
  char entry[10];
  char *str;

  if (de->Value == NULL) return;
  if (de->Type == D_OFF) return;
  SetWindowTextPos(w, de->X + de->Xpos, de->Y);
  if (HighLight) tft.setTextColor(w->Bcolor, w->Fcolor);
  else tft.setTextColor(w->Fcolor, w->Bcolor);
  switch (de->Type)
  {
    case D_STRING:
      p(de->fmt, (char *)de->Value);
      break;
    case D_INT:
      p(de->fmt, *(int *)de->Value);
      break;
    case D_INT8:
      p(de->fmt, *(int8_t *)de->Value);
      break;
    case D_UINT8:
      p(de->fmt, *(uint8_t *)de->Value);
      break;
    case D_FLOAT:
      p(de->fmt, *(float *)de->Value);
      break;
    case D_BINARY8:
      for (int j = 7; j >= 0; j--)
      {
        if (*(uint8_t *)de->Value & (1 << j)) p("1");
        else p("0");
      }
      break;
    case D_YESNO:
      if (*(bool *)de->Value) p("YES");
      else p("NO ");
      break;
    case D_FWDREV:
      if (*(bool *)de->Value) p("FWD");
      else p("REV");
      break;
    case D_ONOFF:
      if (*(bool *)de->Value) p(" ON");
      else p("OFF");
      break;
    case D_OPENCLOSE:
      if (*(bool *)de->Value) p(" OPEN");
      else p("CLOSE");
      break;
    case D_LIST:
      if (de->Value != NULL) p("%*.*s", (int)de->StepSize, (int)de->StepSize, (char *)de->Value);
      break;
    case D_DI:
      entry[0] = * (char *)de->Value;
      entry[1] = 0;
      i = FindInList(DIlist,entry);
      if(i<1) i = 1;
      str = GetEntry(DIlist,i);
      if (str == NULL) break;
      p("%*.*s", (int)de->StepSize, (int)de->StepSize, str);
      break;
    case D_DO:
      entry[0] = * (char *)de->Value;
      entry[1] = 0;
      i = FindInList(DOlist,entry);
      if(i<1) i = 1;
      str = GetEntry(DOlist,i);
      if (str == NULL) break;
      p("%*.*s", (int)de->StepSize, (int)de->StepSize, str);
      break;
    case D_DILEVEL:
      i = (int) * (int8_t *)de->Value + 2;
      if(i<1) i = 1;
      str = GetEntry(DILlist,i);
      if (str == NULL) break;
      p("%*.*s", (int)de->StepSize, (int)de->StepSize, str);
      break;
    case D_DOLEVEL:
      i = (int) * (int8_t *)de->Value + 2;
      if(i<1) i = 1;
      str = GetEntry(DOLlist,i);
      if (str == NULL) break;
      p("%*.*s", (int)de->StepSize, (int)de->StepSize, str);
      break;
    default:
      break;
  }
}

void UpdateNoEditDialogEntries(Window *w, DialogBoxEntry *de)
{
  int   i = 0;

  // Loop through all the DialogBox entries and print the entries
  while (1)
  {
    if (de[i].Name == NULL) return;
    if (de[i].NoEdit == true) DisplayDialogEntry(w, &de[i], false);
    i++;
  }
}

void UpdateNoEditDialogEntries(DialogBox *d)
{
  int   i = 0;

  // Loop through all the DialogBox entries and print the entries
  while (1)
  {
    if (d->Entry[i].Name == NULL) return;
    if (d->Entry[i].NoEdit == true) DisplayDialogEntry(&d->w, &d->Entry[i], false);
    i++;
  }
}

void DisplayAllDialogEntries(DialogBox *d)
{
  int   i = 0;

  // Loop through all the DialogBox entries and print the entries
  while (1)
  {
    if (d->Entry[i].Name == NULL) return;
    //    if(i==d->Selected) DisplayDialogEntry(&d->w, &d->Entry[i++], true);
    //    else DisplayDialogEntry(&d->w, &d->Entry[i++], false);
    DisplayDialogEntry(&d->w, &d->Entry[i++], false);
  }
}

// This function will refresh one entry value per call and then advance to the next
// entry. The idea is to call this function in the polling loop to refesh the display in the
// event of any changes. If an entry is selected it will not be updated.
void RefreshAllDialogEntries(DialogBox *d)
{
  DialogTypes Type;

  while (1)
  {
    if (d->Entry[d->LastUpdated].Name == NULL)
    {
      d->LastUpdated = 0;
      return;
    }
    if ((d->LastUpdated != d->Selected) || (d->State != M_ENTRYSELECTED)) DisplayDialogEntry(&d->w, &d->Entry[d->LastUpdated], false);
    Type = d->Entry[d->LastUpdated++].Type;
    // Exit if its not a type that has no display
    if ((Type != D_MENU) && (Type != D_DIALOG) && (Type != D_TITLE) && (Type != D_FUNCTION)) return;
  }
}

void DisplayDialogEntryNames(Window *w, DialogBoxEntry *de, bool HighLight)
{
  if (de->Name == NULL) return;
  if (de->Type == D_OFF) return;
  if (HighLight) tft.setTextColor(w->Bcolor, w->Fcolor);
  else tft.setTextColor(w->Fcolor, w->Bcolor);
  SetWindowTextPos(w, de->X, de->Y);
  p((char *)de->Name);
}

void PrintDialog(DialogBox *d, int X, int Y, char *text)
{
  tft.setTextColor(d->w.Fcolor, d->w.Bcolor);
  SetWindowTextPos(&d->w, X, Y);
  p(text);
}

// This function displays all the dialog box entry names.
void DisplayAllDialogEntryNames(DialogBox *d)
{
  int   i = 0;

  tft.setTextColor(d->w.Fcolor, d->w.Bcolor);
  // Loop through all the DialogBox entries and print the names if not NULL
  while (1)
  {
    if (d->Entry[i].Name == NULL) return;
    DisplayDialogEntryNames(&d->w, &d->Entry[i++], false);
  }
}

// This function draws the box for the selection dialog box and clears
// the area then displays the initial values with the selected
// option highlighted.
// This is called to initially setup and display a dialog box.
void DialogBoxDisplay(DialogBox *d)
{
  if (d == NULL) return;
  // Draw the window for this dialog box
  DisplayWindow(&d->w);
  // Display all the Dialob box options and highlight the selected option
  DisplayAllDialogEntryNames(d);
  DisplayAllDialogEntries(d);
  // Highlight the selected entry
  if (d->Selected != -1) DisplayDialogEntryNames(&d->w, &d->Entry[d->Selected], true);
  ActiveMenu = NULL;
  ActiveDialog = d;
}

// This function returns a pointer to the requested dialog box entry.
// The entry is specifided by name. The name search is case sensitive.
// A pointer to the dialog box entry is retuned or NULL is not found.
DialogBoxEntry *GetDialogEntries(DialogBoxEntry *de, char *rname)
{
  int   i = 0;

  // Loop through all the DialogBox entries and search if requested the entries
  while (1)
  {
    if (de[i].Name == NULL) return(NULL);
    if (strstr(de[i].Name,rname) != NULL) return(&de[i]);
    i++;
  }
  return(NULL);
}

// The following functions support the dialog box select from a list mode.
// General notes/points about this mode of operation
//   - Use a char string holding a list of comma seperated list names.
//   - *fmt points to the list of names
//   - *Value point to selected name
//   - Number of commas + 1 defnes the number of entries in list
//   Need the following functions:
//    NumListEntries, returns the number of entries in a list
//    GetEntry, returns pointer to the selected entry number
//    FindInList, finds a token in the list and return the index count for the token

// This function determines the number of entries in a list. The entries
// are comma seperated for this function just counts commas and returns the
// count plus 1.
// Zero is returned of the list is a null string or null pointer
int NumListEntries(char *list)
{
  int i = 0, num = 1;

  if (list == NULL) return (0);
  if (list[0] == 0) return (0);
  while (list[i] != 0) if (list[i++] == ',') num++;
  return (num);
}

// This function returns a pointer to a requested entry in a list.
// The list entry is moved to a static string in the function and
// a pointer is returned. The caller should copy this string as required,
// do not assume it will persist, it will be overwritten on the next call
// to this function. null is returned of the entry is not found.
// Max list entry length (token) is 10, this is enforced.
char *GetEntry(char *list, int num)
{
  int i = 0, j = 1;
  static char token[11];

  if (list == NULL) return (NULL);
  if (list[0] == 0) return (NULL);
  // point to entry in list string
  while (j != num)
  {
    if (list[i] == 0) return (NULL);
    if (list[i++] == ',') j++;
  }
  // i indexes to the selected entry.
  if (list[i] == ',') i++;   // skip the inital comma
  token[10] = 0;
  for (j = 0; j < 10; j++)
  {
    token[j] = list[i++];
    if (token[j] == ',') token[j] = 0;
    if (token[j] == 0) break;
  }
  return (token);
}

// This function finds the entry (string) in the list and returns its
// entry number, 1 through max entries. -1 is returned if not found
// or any errors.
int FindInList(char *list, char *entry)
{
  int i, num;
  char *token;

  if (entry == NULL) return -1;
  if ((num = NumListEntries(list)) == 0) return -1;
  for (i = 1; i <= num; i++)
  {
    if ((token = GetEntry(list, i)) == NULL) return -1;
    if (strcmp(token, entry) == 0) return i;
  }
  return -1;
}





