#include <windows.h>
#include <commctrl.h>
#ifdef UNICODE
#include "nsis_unicode/pluginapi.h"
#else
#include "nsis_ansi/pluginapi.h"
#endif
#include "resource.h"

/**
    NSISArray v2.4 by Afrow UK
    A plugin for NSIS which adds support for dynamic arrays.

    Last modified: 22nd April 2010
*/

// #########################################################
// Settings
// #########################################################

// Enable plugin functions
#define ARRAY_FUNC_ERRORSTYLE
#define ARRAY_FUNC_REDIM
#define ARRAY_FUNC_FREEUNUSEDMEM
#define ARRAY_FUNC_ARRAYCOUNT
#define ARRAY_FUNC_ARRAYEXISTS
#define ARRAY_FUNC_WRITE
#define ARRAY_FUNC_WRITELIST
#define ARRAY_FUNC_WRITELISTC
#define ARRAY_FUNC_READ
#define ARRAY_FUNC_READTOSTACK
#define ARRAY_FUNC_SHIFT
#define ARRAY_FUNC_UNSHIFT
#define ARRAY_FUNC_PUSH
#define ARRAY_FUNC_POP
#define ARRAY_FUNC_REVERSE
#define ARRAY_FUNC_CLEAR
#define ARRAY_FUNC_PUT
#define ARRAY_FUNC_SETSIZE
#define ARRAY_FUNC_CUT
#define ARRAY_FUNC_SWAP
#define ARRAY_FUNC_SPLICE
#define ARRAY_FUNC_SORT
#define ARRAY_FUNC_COPY
#define ARRAY_FUNC_JOIN
#define ARRAY_FUNC_EXISTS
#define ARRAY_FUNC_EXISTSI
#define ARRAY_FUNC_SEARCH
#define ARRAY_FUNC_SEARCHI
#define ARRAY_FUNC_SIZEOF
#define ARRAY_FUNC_CONCAT
#define ARRAY_FUNC_SUBTRACT
#define ARRAY_FUNC_SETAUTOREDIM
#define ARRAY_FUNC_DEBUG

// General defines
#define ARRAY_BUFFER_NAME   32   // Max length in TCHARs of an array name
#define ARRAY_BUFFER_INDEX  8    // Length in TCHARs of max array item index
#define ARRAY_MAX_COUNT     8    // Max number of arrays at any one time

#define ARRAY_REDIM_INDEXES_DEFAULT 16   // Allocate X more indexes into memory when auto-redimensioning by default
#define ARRAY_REDIM_BUFFERS_DEFAULT 32   // Allocate X more bytes into memory when auto-redimensioning by default
#define BUFFER_INFO         256  // Length for general output strings

// The NSIS InstFiles page log window control ID
#define IDC_LOGWIN 1016

// #########################################################
// End Settings
// #########################################################

// Error messages
#define MSG_ALREADY_DECLARED TEXT("(1) Array name already declared.")
#define MSG_ERROR            TEXT("(2) Parameters error.")
#define MSG_OUT_OF_RANGE     TEXT("(3) Index out of range.")
#define MSG_NOT_EXISTS       TEXT("(4) Array does not exist.")
#define MSG_LIMIT_REACHED    TEXT("(5) Cannot declare any more arrays.")
#define MSG_ALLOC_FAILED     TEXT("(6) Memory could not be allocated.")

HINSTANCE g_hInstance;

// Declare array stuff
int  g_iArrayCount = 0;
TCHAR g_szArrNames[ARRAY_MAX_COUNT][ARRAY_BUFFER_NAME];

// Array information
#define ARRAY_UPPERINDEX      0
#define ARRAY_INDEXES         1
#define ARRAY_BUFFER          2
#define ARRAY_REDIM_INDEXES   3
#define ARRAY_REDIM_BUFFERS   4
int g_iArrInformation[ARRAY_MAX_COUNT][5];

// Main arrays
TCHAR **g_szArrData[ARRAY_MAX_COUNT];

// This is used to decide whether to use a MessageBox for errors
// and/or use NSIS stack, or use NSIS InstFiles log window.
BOOL bErrorMsgBox = FALSE;
BOOL bErrorStack  = FALSE;
BOOL bErrorLogWin = FALSE;

// Function declarations
BOOL ReDimArray(int iArrayIndex, int iIndexes, int iBufferLen);
int  GetArrayIndexByName(TCHAR *szArrayName);
void ClearStackToEnd();
BOOL my_strstr(TCHAR *s1, int len_s1, TCHAR *s2, int len_s2, BOOL bCaseSensitive);
void my_strcpy(TCHAR *str1, int len1, TCHAR *str2, int len2);
int  my_strlen(TCHAR *str, int len);
//int  my_atoi(TCHAR *p);
#define my_atoi myatoi

#define MALLOC(len) GlobalAlloc(GPTR, len)
#define FREE(ptr) GlobalFree((void*)ptr)
#define NSISFUNC(name) extern "C" void __declspec(dllexport) name(HWND hWndParent, int string_size, TCHAR *variables, stack_t **stacktop, extra_parameters *extra)
BOOL g_bInited;
#define NSISARRAY_INIT() \
{ \
  if (!g_bInited) \
  { \
    EXDLL_INIT(); \
    extra->RegisterPluginCallback(g_hInstance, PluginCallback); \
    g_bInited = TRUE; \
  } \
}

// Plugin callback for new plugin API.
static UINT_PTR PluginCallback(enum NSPIM msg)
{
  return 0;
}

// Function: Displays an error message
void ErrorMsg(HWND hParent, TCHAR *szMsg, TCHAR *szFuncName, int iPushEmpty)
{
  TCHAR szInfo[BUFFER_INFO];
  int i;

  if (iPushEmpty == -1)
    ClearStackToEnd();

  if (bErrorLogWin)
  {
    LVITEM lvi;
    HWND hLogWin = GetDlgItem(FindWindowEx(hParent, NULL, TEXT("#32770"), NULL), IDC_LOGWIN);

    if (hLogWin)
    {
      wsprintf(szInfo, TEXT("NSISArray %s: %s"), szFuncName, szMsg);

      lvi.iItem = ListView_GetItemCount(hLogWin);
      lvi.iSubItem = 0;
      lvi.mask = LVIF_TEXT;
      lvi.pszText = (LPTSTR)szInfo;

      ListView_InsertItem(hLogWin, &lvi);
    }

    for (i=0; i<iPushEmpty; i++)
      pushstring(TEXT(""));
  }
  else
  {

    if (bErrorMsgBox)
    {
      wsprintf(szInfo, TEXT("%s: %s"), szFuncName, szMsg);
      MessageBox(hParent, szInfo, TEXT("NSISArray Error"), MB_OK|MB_ICONSTOP);
    }

    for (i=0; i<iPushEmpty; i++)
      pushstring(TEXT(""));

    if (bErrorStack)
    {
      wsprintf(szInfo, TEXT("%c"), szMsg[1]);
      pushstring(szInfo);
    }

  }
}

// Function: Returns 0 for the error code (no errors).
void NoErrorMsg()
{
  if (bErrorStack)
    pushstring(TEXT("0"));
}

// NSIS Function: Sets which error message style to use
#ifdef ARRAY_FUNC_ERRORSTYLE
NSISFUNC(ErrorStyle)
{
  NSISARRAY_INIT();

  TCHAR szErrorStyle[2];
  popstring(szErrorStyle);

  if (!*szErrorStyle)
  {
    if (bErrorStack && !bErrorMsgBox && bErrorLogWin)
      pushstring(TEXT("5"));
    else if (!bErrorStack && !bErrorMsgBox && bErrorLogWin)
      pushstring(TEXT("4"));
    else if (bErrorStack && bErrorMsgBox && !bErrorLogWin)
      pushstring(TEXT("3"));
    else if (!bErrorStack && bErrorMsgBox && !bErrorLogWin)
      pushstring(TEXT("2"));
    else if (bErrorStack && !bErrorMsgBox && !bErrorLogWin)
      pushstring(TEXT("1"));
    else
      pushstring(TEXT("0"));
  }
  else
  {
    int iErrorStyle = my_atoi(szErrorStyle);

    switch (iErrorStyle)
    {
      case 5:
      {
        bErrorStack  = TRUE;
        bErrorMsgBox = FALSE;
        bErrorLogWin = TRUE;
      }
      break;
      case 4:
      {
        bErrorStack  = FALSE;
        bErrorMsgBox = FALSE;
        bErrorLogWin = TRUE;
      }
      break;
      case 3:
      {
        bErrorStack  = TRUE;
        bErrorMsgBox = TRUE;
        bErrorLogWin = FALSE;
      }
      break;
      case 2:
      {
        bErrorStack  = FALSE;
        bErrorMsgBox = TRUE;
        bErrorLogWin = FALSE;
      }
      break;
      case 1:
      {
        bErrorStack  = TRUE;
        bErrorMsgBox = FALSE;
        bErrorLogWin = FALSE;
      }
      break;
      default:
      {
        bErrorStack  = FALSE;
        bErrorMsgBox = FALSE;
        bErrorLogWin = FALSE;
      }
    }
  }
}
#endif

// NSIS Function: Creates an array
NSISFUNC(New)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szArrayIndexes[8];
  TCHAR szArrayBufferLen[8];

  popstring(szArrayName);
  popstring(szArrayIndexes);
  popstring(szArrayBufferLen);

  if (*szArrayName && *szArrayIndexes && *szArrayBufferLen)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex != -1)
      ErrorMsg(hWndParent, MSG_ALREADY_DECLARED, TEXT("New"), 0);
    else if (g_iArrayCount == (int)ARRAY_MAX_COUNT)
      ErrorMsg(hWndParent, MSG_LIMIT_REACHED, TEXT("New"), 0);
    else
    {
      int iArrayIndexes   = my_atoi(szArrayIndexes);
      int iArrayBufferLen = my_atoi(szArrayBufferLen);
      if ((iArrayIndexes >= 1) && (iArrayBufferLen >= 2))
      {
        g_szArrData[g_iArrayCount] = (TCHAR **)MALLOC(sizeof(TCHAR *) * iArrayIndexes);
        if (g_szArrData[g_iArrayCount] == NULL)
        {
          ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("New"), 0);
          return;
        }

        for (int i=0; i<iArrayIndexes; i++)
        {
          g_szArrData[g_iArrayCount][i] = (TCHAR *)MALLOC(sizeof(TCHAR) * iArrayBufferLen);
          if (g_szArrData[g_iArrayCount][i] == NULL)
          {
            FREE(g_szArrData[g_iArrayCount]);
            ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("New"), 0);
            return;
          }
        }

        g_iArrInformation[g_iArrayCount][ARRAY_UPPERINDEX]    = -1;
        g_iArrInformation[g_iArrayCount][ARRAY_INDEXES]       = iArrayIndexes;
        g_iArrInformation[g_iArrayCount][ARRAY_BUFFER]        = iArrayBufferLen;
        g_iArrInformation[g_iArrayCount][ARRAY_REDIM_INDEXES] = ARRAY_REDIM_INDEXES_DEFAULT;
        g_iArrInformation[g_iArrayCount][ARRAY_REDIM_BUFFERS] = ARRAY_REDIM_BUFFERS_DEFAULT;

        my_strcpy(g_szArrNames[g_iArrayCount], (int)ARRAY_BUFFER_NAME, szArrayName, (int)ARRAY_BUFFER_NAME);
        g_iArrayCount++;

        NoErrorMsg();
      }
      else
        ErrorMsg(hWndParent, MSG_ERROR, TEXT("New"), 0);
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("New"), 0);
}

// NSIS Function: Sets auto-redimensioning indexes and string lengths to increase by
#ifdef ARRAY_FUNC_SETAUTOREDIM
NSISFUNC(SetAutoReDim)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szArrayIndexes[8];
  TCHAR szArrayBufferLen[8];

  popstring(szArrayName);
  popstring(szArrayIndexes);
  popstring(szArrayBufferLen);

  if (*szArrayName && *szArrayIndexes && *szArrayBufferLen)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("SetAutoReDim"), 0);
    else
    {
      int iArrayIndexes   = my_atoi(szArrayIndexes),
          iArrayBufferLen = my_atoi(szArrayBufferLen);

      if (iArrayIndexes >= 8 && iArrayBufferLen >= 8)
      {
        g_iArrInformation[iArrayIndex][ARRAY_REDIM_INDEXES] = iArrayIndexes;
        g_iArrInformation[iArrayIndex][ARRAY_REDIM_BUFFERS] = iArrayBufferLen;
      }
      else
        ErrorMsg(hWndParent, MSG_ERROR, TEXT("SetAutoReDim"), 0);
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("SetAutoReDim"), 0);
}
#endif

// NSIS Function: Re-dimensions an array in indexes and buffer lengths
#ifdef ARRAY_FUNC_REDIM
NSISFUNC(ReDim)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szArrayIndexes[8];
  TCHAR szArrayBufferLen[8];

  popstring(szArrayName);
  popstring(szArrayIndexes);
  popstring(szArrayBufferLen);

  if (*szArrayName && *szArrayIndexes && *szArrayBufferLen)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
        ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("ReDim"), 0);
    else
    {
      int iArrayIndexes   = my_atoi(szArrayIndexes),
          iArrayBufferLen = my_atoi(szArrayBufferLen);
      if ((iArrayIndexes >= 1) && (iArrayBufferLen >= 2))
      {
        int i;
        TCHAR **szArrTemp = (TCHAR **)MALLOC(sizeof(TCHAR *) * iArrayIndexes);
        if (szArrTemp == NULL)
        {
          ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("ReDim"), 0);
          return;
        }

        for (i=0; i<iArrayIndexes; i++)
        {
          szArrTemp[i] = (TCHAR *)MALLOC(sizeof(TCHAR) * iArrayBufferLen);
          if (szArrTemp[i] == NULL)
          {
            FREE(szArrTemp);
            ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("ReDim"), 0);
            return;
          }
        }

        for (i=0; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        {
          if (i == iArrayIndexes)
            break;
          my_strcpy(szArrTemp[i], iArrayBufferLen, g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);
        }

        for (i=0; i<g_iArrInformation[iArrayIndex][ARRAY_INDEXES]; i++)
          FREE(g_szArrData[iArrayIndex][i]);
        FREE(g_szArrData[iArrayIndex]);

        g_szArrData[iArrayIndex] = szArrTemp;
        g_iArrInformation[iArrayIndex][ARRAY_INDEXES] = iArrayIndexes;
        g_iArrInformation[iArrayIndex][ARRAY_BUFFER]  = iArrayBufferLen;

        NoErrorMsg();
      }
      else
        ErrorMsg(hWndParent, MSG_ERROR, TEXT("ReDim"), 0);
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("ReDim"), 0);
}
#endif

// NSIS Function: Frees empty indexes from end of array and trims memory to fit longest string
#ifdef ARRAY_FUNC_FREEUNUSEDMEM
NSISFUNC(FreeUnusedMem)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("FreeUnusedMem"), 0);
    else
    {
      int i, j, iArrayItems, iArrayStrLen=0;
      TCHAR **szArrTemp;

      iArrayItems = g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]+1;
      for (i=0; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
      {
        j = my_strlen(g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);
        if (iArrayStrLen < j)
          iArrayStrLen = j;
      }

      szArrTemp = (TCHAR **)MALLOC(sizeof(TCHAR *) * iArrayItems);
      if (szArrTemp == NULL)
      {
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("FreeUnusedMem"), 0);
        return;
      }

      for (i=0; i<iArrayItems; i++)
      {
        szArrTemp[i] = (TCHAR *)MALLOC(sizeof(TCHAR) * iArrayStrLen);
        if (szArrTemp[i] == NULL)
        {
          FREE(szArrTemp);
          ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("FreeUnusedMem"), 0);
          return;
        }
      }

      for (i=0; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        my_strcpy(szArrTemp[i], iArrayStrLen, g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);

      for (i=0; i<g_iArrInformation[iArrayIndex][ARRAY_INDEXES]; i++)
        FREE(g_szArrData[iArrayIndex][i]);
      FREE(g_szArrData[iArrayIndex]);

      g_szArrData[iArrayIndex] = szArrTemp;
      g_iArrInformation[iArrayIndex][ARRAY_INDEXES] = iArrayItems;
      g_iArrInformation[iArrayIndex][ARRAY_BUFFER] = iArrayStrLen;

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("FreeUnusedMem"), 0);
}
#endif

// NSIS Function: Returns the number of arrays created
#ifdef ARRAY_FUNC_ARRAYCOUNT
NSISFUNC(ArrayCount)
{
  NSISARRAY_INIT();

  TCHAR buff[8];
  wsprintf(buff, TEXT("%i"), g_iArrayCount);
  pushstring(buff);
}
#endif

// NSIS Function: Checks if array has been created
#ifdef ARRAY_FUNC_ARRAYEXISTS
NSISFUNC(ArrayExists)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      pushstring(TEXT("0"));
    else
      pushstring(TEXT("1"));

    NoErrorMsg();
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("ArrayExists"), 0);
}
#endif

// NSIS Function: Writes a value to an array
#ifdef ARRAY_FUNC_WRITE
NSISFUNC(Write)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Write"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szIndex);

  if (*szArrayName && *szIndex && popstring(szValue) == 0)
  {
    int iIndex = my_atoi(szIndex);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Write"), 0);
    else if (((iIndex >= 0) && (iIndex <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]+1)) || ((g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] == -1) && (iIndex == 0)))
    {
      if (!ReDimArray(iArrayIndex, 1, my_strlen(szValue, string_size)))
      {
        FREE(szValue);
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Write"), 0);
        return;
      }
      my_strcpy(g_szArrData[iArrayIndex][iIndex], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);

      if (iIndex == g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]+1)
        g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]++;

      NoErrorMsg();
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Write"), 0);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Write"), 0);

  FREE(szValue);
}
#endif

// NSIS Function: Writes a list of values to an array
#ifdef ARRAY_FUNC_WRITELIST
NSISFUNC(WriteList)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("WriteList"), 0);
    return;
  }

  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("WriteList"), -1);
    else
    {
      while ((popstring(szValue) == 0) && (lstrcmpi(szValue, TEXT("/END")) != 0))
      {
        if (!ReDimArray(iArrayIndex, 1, my_strlen(szValue, string_size)))
        {
          FREE(szValue);
          ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("WriteList"), 0);
          return;
        }
        g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]++;
        my_strcpy(g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);
      }

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("WriteList"), -1);

  FREE(szValue);
}
#endif

// NSIS Function: Writes a list of values separated by a TCHARacter to an array
#ifdef ARRAY_FUNC_WRITELISTC
NSISFUNC(WriteListC)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szSeparator[2];
  TCHAR *szValue;
  TCHAR *szOut;
  if ((szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size)) == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("WriteListC"), 0);
    return;
  }
  if ((szOut = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size)) == NULL)
  {
    FREE(szValue);
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("WriteListC"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szOut);
  popstring(szSeparator);

  if (*szArrayName && *szOut && *szSeparator)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("WriteListC"), 0);
    else
    {
      int i = 0, j = 0;
      while (i<string_size)
      {
        if ((szOut[i] == szSeparator[0]) || (szOut[i] == '\0'))
        {
          szValue[j] = '\0';

          if (!ReDimArray(iArrayIndex, 1, my_strlen(szValue, string_size)))
          {
            ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("WriteListC"), 0);
            FREE(szValue);
            return;
          }
          g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]++;

          my_strcpy(g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);

          j = 0;
          if (szOut[i] == '\0')
            break;
        }
        else
        {
          szValue[j] = szOut[i];
          j++;
        }
        i++;
      }
      NoErrorMsg();
    }
  } 
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("WriteListC"), 0);

  FREE(szSeparator);
  FREE(szOut);
}
#endif

// NSIS Function: Reads a value from array
#ifdef ARRAY_FUNC_READ
NSISFUNC(Read)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];

  popstring(szArrayName);
  popstring(szIndex);

  if (*szArrayName && *szIndex)
  {
    int iIndex   = my_atoi(szIndex);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Read"), 1);
    else if ((iIndex >= 0) && (iIndex <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]) && (g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] != -1))
    {
      pushstring(g_szArrData[iArrayIndex][iIndex]);
      NoErrorMsg();
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Read"), 1);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Read"), 1);
}
#endif

// NSIS Function: Reads all values in array and places them onto the NSIS stack
#ifdef ARRAY_FUNC_READTOSTACK
NSISFUNC(ReadToStack)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndexFrom[ARRAY_BUFFER_INDEX];
  TCHAR szIndexTo[ARRAY_BUFFER_INDEX];

  popstring(szArrayName);
  popstring(szIndexFrom);
  popstring(szIndexTo);

  if (*szArrayName && *szIndexFrom && *szIndexTo)
  {
    int iIndexFrom = my_atoi(szIndexFrom);
    int iIndexTo   = my_atoi(szIndexTo);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("ReadToStack"), 0);
    else if ((iIndexTo > g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]) || (iIndexTo < ((g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]+1)*(-1))) || (iIndexFrom < 0) || (iIndexFrom > g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]) || (g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] == -1))
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("ReadToStack"), 0);
    else
    {
      int i;
      if (iIndexTo < 0)
        iIndexTo = g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] + 1 + iIndexTo;

      for (i=iIndexTo; i>=iIndexFrom; i--)
        pushstring(g_szArrData[iArrayIndex][i]);

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("ReadToStack"), 0);
}
#endif

// NSIS Function: Pushes a value to end of array
#ifdef ARRAY_FUNC_SHIFT
NSISFUNC(Shift)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Shift"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szValue);

  if (*szArrayName && *szValue)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Shift"), 0);
    else
    {
      if (!ReDimArray(iArrayIndex, 1, my_strlen(szValue, string_size)))
      {
        FREE(szValue);
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Shift"), 0);
        return;
      }
      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]++;
      my_strcpy(g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Shift"), 0);

  FREE(szValue);
}
#endif

// NSIS Function: Pops a value from end of array
#ifdef ARRAY_FUNC_UNSHIFT
NSISFUNC(Unshift)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Unshift"), 1);
    else if (g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] == -1)
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Unshift"), 1);
    else
    {
      pushstring(g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]]);
      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]--;

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Unshift"), 1);
}
#endif

// NSIS Function: Pushes a value to front of array
#ifdef ARRAY_FUNC_PUSH
NSISFUNC(Push)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Push"), 0);
    return;
  }

  popstring(szArrayName);

  if (*szArrayName && popstring(szValue) == 0)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Push"), 0);
    else
    {
      int i;
      TCHAR *ptr;
      if (!ReDimArray(iArrayIndex, 1, my_strlen(szValue, string_size)))
      {
        FREE(szValue);
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Push"), 0);
        return;
      }

      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]++;
      ptr = g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]];

      for (i=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i>0; i--)
        g_szArrData[iArrayIndex][i] = g_szArrData[iArrayIndex][i-1];

      g_szArrData[iArrayIndex][ARRAY_UPPERINDEX] = ptr;
      my_strcpy(g_szArrData[iArrayIndex][0], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Push"), 0);

  FREE(szValue);
}
#endif

// NSIS Function: Pops a value from front of array
#ifdef ARRAY_FUNC_POP
NSISFUNC(Pop)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Pop"), 1);
    else if (g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] == -1)
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Pop"), 1);
    else
    {
      int i;
      TCHAR *ptr;

      pushstring(g_szArrData[iArrayIndex][0]);
      ptr = g_szArrData[iArrayIndex][0];

      for (i=0; i<g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        g_szArrData[iArrayIndex][i] = g_szArrData[iArrayIndex][i+1];

      g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]] = ptr;
      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]--;

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Pop"), 1);
}
#endif

// NSIS Function: Reverses the array
#ifdef ARRAY_FUNC_REVERSE
NSISFUNC(Reverse)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Reverse"), 0);
    else
    {
      int i, j;
      TCHAR *ptr;

      for (i=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX],j=0; i>j; i--,j++)
      {
        ptr = g_szArrData[iArrayIndex][i];
        g_szArrData[iArrayIndex][i] = g_szArrData[iArrayIndex][j];
        g_szArrData[iArrayIndex][j] = ptr;
      }

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Reverse"), 0);
}
#endif

// NSIS Function: Removes all items in array
#ifdef ARRAY_FUNC_CLEAR
NSISFUNC(Clear)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Clear"), 0);
    else
    {
      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] = -1;

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Clear"), 0);
}
#endif

// NSIS Function: Deletes array
NSISFUNC(Delete)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Delete"), 0);
    else
    {
      int i, j;

      for (i=0; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        FREE(g_szArrData[iArrayIndex][i]);
      FREE(g_szArrData[iArrayIndex]);

      for (i=iArrayIndex; i<g_iArrayCount-1; i++)
      {
        j = i + 1;
        g_szArrData[i] = g_szArrData[j];
        my_strcpy(g_szArrNames[i], (int)ARRAY_BUFFER_NAME, g_szArrNames[j], (int)ARRAY_BUFFER_NAME);
        g_iArrInformation[i][ARRAY_UPPERINDEX] = g_iArrInformation[j][ARRAY_UPPERINDEX];
        g_iArrInformation[i][ARRAY_INDEXES]    = g_iArrInformation[j][ARRAY_INDEXES];
        g_iArrInformation[i][ARRAY_BUFFER]     = g_iArrInformation[j][ARRAY_BUFFER];
      }
      
      g_iArrayCount--;
      g_iArrInformation[g_iArrayCount][ARRAY_UPPERINDEX] = g_iArrInformation[g_iArrayCount][ARRAY_INDEXES] = g_iArrInformation[g_iArrayCount][ARRAY_BUFFER] = 0;
      my_strcpy(g_szArrNames[g_iArrayCount], (int)ARRAY_BUFFER_NAME, TEXT(""), 1);

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Delete"), 0);
}

// NSIS Function: Puts value into array at index
#ifdef ARRAY_FUNC_PUT
NSISFUNC(Put)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Put"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szIndex);

  if (*szArrayName && *szIndex && popstring(szValue) == 0)
  {
    int iIndex = my_atoi(szIndex);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Put"), 0);
    else if (((iIndex >= 0) && (iIndex <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX])) || ((g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] == -1) && (iIndex == -1)))
    {
      int i;
      TCHAR *ptr;
      if (!ReDimArray(iArrayIndex, 1, my_strlen(szValue, string_size)))
      {
        FREE(szValue);
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Put"), 0);
        return;
      }

      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]++;
      ptr = g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]];

      for (i=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i>iIndex; i--)
        g_szArrData[iArrayIndex][i] = g_szArrData[iArrayIndex][i-1];

      g_szArrData[iArrayIndex][iIndex] = ptr;
      my_strcpy(g_szArrData[iArrayIndex][iIndex], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);

      NoErrorMsg();
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Put"), 0);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Put"), 0);

  FREE(szValue);
}
#endif

// NSIS Function: Adds or removes items to array to make it size
#ifdef ARRAY_FUNC_SETSIZE
NSISFUNC(SetSize)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndexes[ARRAY_BUFFER_INDEX];

  popstring(szArrayName);
  popstring(szIndexes);

  if (*szArrayName && *szIndexes)
  {
    int iIndexes = my_atoi(szIndexes);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("SetSize"), 0);
    else if ((iIndexes >= 0) && (iIndexes < g_iArrInformation[iArrayIndex][ARRAY_INDEXES]))
    {
      int i;
      if (iIndexes > g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX])
      {
        for (i=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]+1; i<iIndexes; i++)
          my_strcpy(g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], TEXT(""), 1);
      }
      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] = iIndexes-1;

      NoErrorMsg();
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("SetSize"), 0);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("SetSize"), 0);
}
#endif

// NSIS Function: Cuts value out of array at index
#ifdef ARRAY_FUNC_CUT
NSISFUNC(Cut)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];

  popstring(szArrayName);
  popstring(szIndex);

  if (*szArrayName && *szIndex)
  {
    int iIndex = my_atoi(szIndex);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Cut"), 1);
    else if (((iIndex >= 0) && (iIndex <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX])) || ((g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] == -1) && (iIndex == 0)))
    {
      int i;
      TCHAR *ptr;

      pushstring(g_szArrData[iArrayIndex][iIndex]);
      ptr = g_szArrData[iArrayIndex][iIndex];

      for (i=iIndex; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        g_szArrData[iArrayIndex][i] = g_szArrData[iArrayIndex][i+1];

      g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]] = ptr;

      g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]--;

      NoErrorMsg();
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Cut"), 1);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Cut"), 1);
}
#endif

// NSIS Function: Swap items at two indexes
#ifdef ARRAY_FUNC_SWAP
NSISFUNC(Swap)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndexA[ARRAY_BUFFER_INDEX];
  TCHAR szIndexB[ARRAY_BUFFER_INDEX];

  popstring(szArrayName);
  popstring(szIndexA);
  popstring(szIndexB);

  if (*szArrayName && *szIndexA && *szIndexB)
  {
    int iIndexA = my_atoi(szIndexA);
    int iIndexB = my_atoi(szIndexB);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Swap"), 0);
    else if ((iIndexA >= 0) && (iIndexB >= 0) && ((iIndexA <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]) && (iIndexB <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX])) && ((iIndexA >= 0) && (iIndexB >= 0)) && (g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] != -1))
    {
      TCHAR *ptr;

      ptr = g_szArrData[iArrayIndex][iIndexB];
      g_szArrData[iArrayIndex][iIndexB] = g_szArrData[iArrayIndex][iIndexA];
      g_szArrData[iArrayIndex][iIndexA] = ptr;

      NoErrorMsg();
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Swap"), 0);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Swap"), 0);
}
#endif

// NSIS Function: Place an array of values into an array
#ifdef ARRAY_FUNC_SPLICE
NSISFUNC(Splice)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndexFrom[ARRAY_BUFFER_INDEX];
  TCHAR szIndexTo[ARRAY_BUFFER_INDEX];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Splice"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szIndexFrom);
  popstring(szIndexTo);

  if (*szArrayName && *szIndexFrom && *szIndexTo)
  {
    int iIndexFrom = my_atoi(szIndexFrom);
    int iIndexTo   = my_atoi(szIndexTo);
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Splice"), -1);
    else if (
             (iIndexFrom >= 0) && 
            ((iIndexFrom <= iIndexTo) || (iIndexTo == -1)) && 
            ((iIndexFrom <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]) && (iIndexTo <= g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX])) && 
            ((iIndexFrom >= 0) && (iIndexTo >= -1)) && 
            (g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] != -1)
            )
    {
      int i, j=0, k=0;
      TCHAR *ptr;

      if (iIndexTo < 0)
        iIndexTo = g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] + 1 + iIndexTo;

      while ((popstring(szValue) == 0) && (lstrcmpi(szValue, TEXT("/END")) != 0))
      {
        k = iIndexFrom + j;
        if (k > iIndexTo)
        {
          if (!ReDimArray(iArrayIndex, 1, my_strlen(szValue, string_size)))
          {
            FREE(szValue);
            ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Splice"), 0);
            return;
          }

          g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]++;
          ptr = g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]];

          for (i=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i>k; i--)
            g_szArrData[iArrayIndex][i] = g_szArrData[iArrayIndex][i-1];

          g_szArrData[iArrayIndex][k] = ptr;
          my_strcpy(g_szArrData[iArrayIndex][k], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);
        }
        else
          my_strcpy(g_szArrData[iArrayIndex][k], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size);
        j++;
      }

      k = iIndexTo - iIndexFrom + 1;
      if (j <= k)
      {
        for (i=iIndexFrom+k-1; i>=iIndexFrom+j; i--)
        {
          ptr = g_szArrData[iArrayIndex][i];

          for (k=i; k<g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; k++)
            g_szArrData[iArrayIndex][k] = g_szArrData[iArrayIndex][k+1];

          g_szArrData[iArrayIndex][g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]] = ptr;

          g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]--;
        }
      }

      NoErrorMsg();
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Splice"), -1);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Splice"), -1);

  FREE(szValue);
}
#endif

// NSIS Function: Sorts the array alphabetically
#ifdef ARRAY_FUNC_SORT
NSISFUNC(Sort)
{
  NSISARRAY_INIT();

  TCHAR szArrayNameA[ARRAY_BUFFER_NAME];
  TCHAR szArrayNameB[ARRAY_BUFFER_NAME];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Sort"), 0);
    return;
  }

  BOOL bNumeric = FALSE;

  popstring(szArrayNameA);
  if (*szArrayNameA && lstrcmpi(szArrayNameA, TEXT("/numeric")) == 0)
  {
    bNumeric = TRUE;
    popstring(szArrayNameA);
  }
  popstring(szArrayNameB);

  if (*szArrayNameA)
  {
    int iArrayIndexA = GetArrayIndexByName(szArrayNameA);
    int iArrayIndexB = -1;
    if (*szArrayNameB)
      iArrayIndexB = GetArrayIndexByName(szArrayNameB);
    if ((iArrayIndexB != -1) && (g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX] != g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX]))
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Sort"), 0);
    else if (iArrayIndexA == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Sort"), 0);
    else
    {
      int i;
      TCHAR *ptr;

      for (i=0; i<=g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX]; i++)
      {
        int j;
        for (j=i+1; j<=g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX]; j++)
        {
          if ((bNumeric && my_atoi(g_szArrData[iArrayIndexA][i]) > my_atoi(g_szArrData[iArrayIndexA][j])) ||
            (!bNumeric && lstrcmp(g_szArrData[iArrayIndexA][i], g_szArrData[iArrayIndexA][j]) > 0))
          {
            ptr = g_szArrData[iArrayIndexA][i];
            g_szArrData[iArrayIndexA][i] = g_szArrData[iArrayIndexA][j];
            g_szArrData[iArrayIndexA][j] = ptr;
            if (iArrayIndexB != -1)
            {
              ptr = g_szArrData[iArrayIndexB][i];
              g_szArrData[iArrayIndexB][i] = g_szArrData[iArrayIndexB][j];
              g_szArrData[iArrayIndexB][j] = ptr;
            }
          }
        }
      }

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Sort"), 0);

  FREE(szValue);
}
#endif

// NSIS Function: Copies entire contents of an array to another array
#ifdef ARRAY_FUNC_COPY
NSISFUNC(Copy)
{
  NSISARRAY_INIT();

  TCHAR szArrayNameA[ARRAY_BUFFER_NAME];
  TCHAR szArrayNameB[ARRAY_BUFFER_NAME];

  popstring(szArrayNameA);
  popstring(szArrayNameB);

  if (*szArrayNameA && *szArrayNameB)
  {
    int iArrayIndexA = GetArrayIndexByName(szArrayNameA);
    int iArrayIndexB = GetArrayIndexByName(szArrayNameB);
    if ((iArrayIndexA == -1) || (iArrayIndexB == -1))
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Copy"), 0);
    else
    {
      int i;
      TCHAR **szArrTemp = (TCHAR **)MALLOC(sizeof(TCHAR *) * g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX]+1);
      if (szArrTemp == NULL)
      {
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Copy"), 0);
        return;
      }

      for (i=0; i<=g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX]; i++)
      {
        szArrTemp[i] = (TCHAR *)MALLOC(sizeof(TCHAR) * g_iArrInformation[iArrayIndexA][ARRAY_BUFFER]);
        if (szArrTemp[i] == NULL)
        {
          FREE(szArrTemp);
          ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Copy"), 0);
          return;
        }
        my_strcpy(szArrTemp[i], g_iArrInformation[iArrayIndexA][ARRAY_BUFFER], g_szArrData[iArrayIndexA][i], g_iArrInformation[iArrayIndexA][ARRAY_BUFFER]);
      }

      for (i=0; i<=g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX]; i++)
        FREE(g_szArrData[iArrayIndexB][i]);
      FREE(g_szArrData[iArrayIndexB]);

      g_szArrData[iArrayIndexB] = szArrTemp;

      g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX] = g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX];
      g_iArrInformation[iArrayIndexB][ARRAY_INDEXES] = g_iArrInformation[iArrayIndexA][ARRAY_INDEXES];
      g_iArrInformation[iArrayIndexB][ARRAY_BUFFER] = g_iArrInformation[iArrayIndexA][ARRAY_BUFFER];

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Copy"), 0);
}
#endif

// NSIS Function: Joins two arrays
#ifdef ARRAY_FUNC_JOIN
NSISFUNC(Join)
{
  NSISARRAY_INIT();

  TCHAR szArrayNameA[ARRAY_BUFFER_NAME];
  TCHAR szArrayNameB[ARRAY_BUFFER_NAME];

  popstring(szArrayNameA);
  popstring(szArrayNameB);

  if (*szArrayNameA && *szArrayNameB)
  {
    int iArrayIndexA = GetArrayIndexByName(szArrayNameA);
    int iArrayIndexB = GetArrayIndexByName(szArrayNameB);
    if ((iArrayIndexA == -1) || (iArrayIndexB == -1))
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Join"), 0);
    else
    {
      int i, j, iNewSize = g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX] + g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX] + 2;
      TCHAR **szArrTemp = (TCHAR **)MALLOC(sizeof(TCHAR *) * iNewSize);
      if (szArrTemp == NULL)
      {
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Join"), 0);
        return;
      }

      for (i=0; i<iNewSize; i++)
      {
        szArrTemp[i] = (TCHAR *)MALLOC(sizeof(TCHAR) * g_iArrInformation[iArrayIndexA][ARRAY_BUFFER]);
        if (szArrTemp[i] == NULL)
        {
          FREE(szArrTemp);
          ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Join"), 0);
          return;
        }
      }

      for (i=0; i<=g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX]; i++)
        my_strcpy(szArrTemp[i], g_iArrInformation[iArrayIndexA][ARRAY_BUFFER], g_szArrData[iArrayIndexB][i], g_iArrInformation[iArrayIndexB][ARRAY_BUFFER]);

      for (i=g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX]+1, j=0; i<iNewSize; i++, j++)
        my_strcpy(szArrTemp[i], g_iArrInformation[iArrayIndexA][ARRAY_BUFFER], g_szArrData[iArrayIndexA][j], g_iArrInformation[iArrayIndexA][ARRAY_BUFFER]);

      for (i=0; i<g_iArrInformation[iArrayIndexB][ARRAY_INDEXES]; i++)
        FREE(g_szArrData[iArrayIndexB][i]);
      FREE(g_szArrData[iArrayIndexB]);

      g_szArrData[iArrayIndexB] = szArrTemp;

      g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX] = iNewSize-1;
      g_iArrInformation[iArrayIndexB][ARRAY_INDEXES] = iNewSize;
      g_iArrInformation[iArrayIndexB][ARRAY_BUFFER] = g_iArrInformation[iArrayIndexA][ARRAY_BUFFER];

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Join"), 0);
}
#endif

// NSIS Function: Checks if array value exists
#ifdef ARRAY_FUNC_EXISTS
NSISFUNC(Exists)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Exists"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szValue);
  popstring(szIndex);

  if (*szArrayName && *szValue && *szIndex)
  {
    int iIndex = my_atoi(szIndex);
    if (iIndex >= 0)
    {
      int iArrayIndex = GetArrayIndexByName(szArrayName);
      if (iArrayIndex == -1)
        ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Exists"), 1);
      else
      {
        int i;
        BOOL bFound = FALSE;

        for (i=iIndex; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        {
          if (lstrcmp(g_szArrData[iArrayIndex][i], szValue) == 0)
          {
            wsprintf(szIndex, TEXT("%i"), i);
            pushstring(szIndex);
            bFound = TRUE;
            break;
          }
        }

        if (bFound == FALSE)
          pushstring(TEXT("-1"));

        NoErrorMsg();
      }
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Exists"), 1);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Exists"), 1);

  FREE(szValue);
}
#endif

// NSIS Function: Checks if array value exists (case insensitive)
#ifdef ARRAY_FUNC_EXISTSI
NSISFUNC(ExistsI)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("ExistsI"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szValue);
  popstring(szIndex);

  if (*szArrayName && *szValue && *szIndex)
  {
    int iIndex = my_atoi(szIndex);
    if (iIndex >= 0)
    {
      int iArrayIndex = GetArrayIndexByName(szArrayName);
      if (iArrayIndex == -1)
        ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("ExistsI"), 1);
      else
      {
        int i;
        BOOL bFound = FALSE;

        for (i=iIndex; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        {
          if (lstrcmpi(g_szArrData[iArrayIndex][i], szValue) == 0)
          {
            wsprintf(szIndex, TEXT("%i"), i);
            pushstring(szIndex);
            bFound = TRUE;
            break;
          }
        }

        if (!bFound)
          pushstring(TEXT("-1"));

        NoErrorMsg();
      }
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("ExistsI"), 1);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("ExistsI"), 1);

  FREE(szValue);
}
#endif

// NSIS Function: Search array for value (case sensitive)
#ifdef ARRAY_FUNC_SEARCH
NSISFUNC(Search)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Search"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szValue);
  popstring(szIndex);

  if (*szArrayName && *szValue && *szIndex)
  {
    int iIndex = my_atoi(szIndex);
    if (iIndex >= 0)
    {
      int iArrayIndex = GetArrayIndexByName(szArrayName);
      if (iArrayIndex == -1)
        ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Search"), 1);
      else
      {
        int i;
        BOOL bFound = FALSE;

        for (i=iIndex; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        {
          if (my_strstr(g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size, TRUE))
          {
            wsprintf(szIndex, TEXT("%i"), i);
            pushstring(szIndex);
            bFound = TRUE;
            break;
          }
        }

        if (!bFound)
          pushstring(TEXT("-1"));

        NoErrorMsg();
      }
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("Search"), 1);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Search"), 1);

  FREE(szValue);
}
#endif

// NSIS Function: Search array for value (case insensitive)
#ifdef ARRAY_FUNC_SEARCHI
NSISFUNC(SearchI)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR szIndex[ARRAY_BUFFER_INDEX];
  TCHAR *szValue = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size);
  if (szValue == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("SearchI"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szValue);
  popstring(szIndex);

  if (*szArrayName && *szValue && *szIndex)
  {
    int iIndex = my_atoi(szIndex);
    if (iIndex >= 0)
    {
      int iArrayIndex = GetArrayIndexByName(szArrayName);
      if (iArrayIndex == -1)
        ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("SearchI"), 1);
      else
      {
        int i;
        BOOL bFound = FALSE;

        for (i=iIndex; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
        {
          if (my_strstr(g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER], szValue, string_size, FALSE))
          {
            wsprintf(szIndex, TEXT("%i"), i);
            pushstring(szIndex);
            bFound = TRUE;
            break;
          }
        }

        if (!bFound)
          pushstring(TEXT("-1"));

        NoErrorMsg();
      }
    }
    else
      ErrorMsg(hWndParent, MSG_OUT_OF_RANGE, TEXT("SearchI"), 1);
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("SearchI"), 1);

  FREE(szValue);
}
#endif

// NSIS Function: Get size of array
#ifdef ARRAY_FUNC_SIZEOF
NSISFUNC(SizeOf)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];

  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("SizeOf"), 3);
    else
    {
      TCHAR szSize[8];
      wsprintf(szSize, TEXT("%i"), g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]+1);
      pushstring(szSize);
      wsprintf(szSize, TEXT("%i"), g_iArrInformation[iArrayIndex][ARRAY_INDEXES]);
      pushstring(szSize);
      wsprintf(szSize, TEXT("%i"), g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);
      pushstring(szSize);

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("SizeOf"), 3);
}
#endif

// NSIS Function: Join all items in array together + output it
#ifdef ARRAY_FUNC_CONCAT
NSISFUNC(Concat)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  TCHAR *szOut;
  TCHAR *szJoinChar;
  if ((szOut = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size)) == NULL)
  {
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Concat"), 0);
    return;
  }
  if ((szJoinChar = (TCHAR *)MALLOC(sizeof(TCHAR) * string_size)) == NULL)
  {
    FREE(szOut);
    ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Concat"), 0);
    return;
  }

  popstring(szArrayName);
  popstring(szJoinChar);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Concat"), 1);
    else
    {
      int i, iLen = 0;
      my_strcpy(szOut, string_size, g_szArrData[iArrayIndex][0], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);
      iLen = my_strlen(g_szArrData[iArrayIndex][0], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);

      for (i=1; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
      {
        iLen += my_strlen(g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]) + 1;
        if (iLen >= string_size)
          break;

        lstrcat(szOut, szJoinChar);
        lstrcat(szOut, g_szArrData[iArrayIndex][i]);
      }

      pushstring(szOut);

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Concat"), 1);

  FREE(szJoinChar);
  FREE(szOut);
}
#endif

// NSIS Function: Subtract items from one array that are in another
#ifdef ARRAY_FUNC_SUBTRACT
NSISFUNC(Subtract)
{
  NSISARRAY_INIT();

  TCHAR szArrayNameA[ARRAY_BUFFER_NAME];
  TCHAR szArrayNameB[ARRAY_BUFFER_NAME];

  popstring(szArrayNameA);
  popstring(szArrayNameB);

  if (*szArrayNameA && *szArrayNameB)
  {
    int iArrayIndexA = GetArrayIndexByName(szArrayNameA);
    int iArrayIndexB = GetArrayIndexByName(szArrayNameB);
    if ((iArrayIndexA == -1) || (iArrayIndexB == -1))
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Subtract"), 1);
    else
    {
      int i, j, k = 0, iNewSize = 0;
      BOOL bFound;
      TCHAR **szArrTemp;

      for (i=0; i<=g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX]; i++)
      {
        for (j=0; j<=g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX]; j++)
        {
          if (lstrcmp(g_szArrData[iArrayIndexA][i], g_szArrData[iArrayIndexB][j]) == 0)
          {
            iNewSize++;
            break;
          }
        }
      }

      szArrTemp = (TCHAR **)MALLOC(sizeof(TCHAR *) * iNewSize);
      if (szArrTemp == NULL)
      {
        ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Subtract"), 0);
        return;
      }

      for (i=0; i<iNewSize; i++)
      {
        szArrTemp[i] = (TCHAR *)MALLOC(sizeof(TCHAR) * g_iArrInformation[iArrayIndexA][ARRAY_BUFFER]);
        if (szArrTemp[i] == NULL)
        {
          FREE(szArrTemp);
          ErrorMsg(hWndParent, MSG_ALLOC_FAILED, TEXT("Subtract"), 0);
          return;
        }
      }

      for (i=0; i<=g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX]; i++)
      {
        bFound = FALSE;
        for (j=0; j<=g_iArrInformation[iArrayIndexB][ARRAY_UPPERINDEX]; j++)
        {
          if (lstrcmp(g_szArrData[iArrayIndexA][i], g_szArrData[iArrayIndexB][j]) == 0)
          {
            bFound = TRUE;
            break;
          }
        }
        if (!bFound)
        {
          my_strcpy(szArrTemp[k], g_iArrInformation[iArrayIndexA][ARRAY_BUFFER], g_szArrData[iArrayIndexA][i], g_iArrInformation[iArrayIndexA][ARRAY_BUFFER]);
          k++;
        }
      }

      for (i=0; i<g_iArrInformation[iArrayIndexA][ARRAY_INDEXES]; i++)
        FREE(g_szArrData[iArrayIndexA][i]);
      FREE(g_szArrData[iArrayIndexA]);

      g_szArrData[iArrayIndexA] = szArrTemp;

      g_iArrInformation[iArrayIndexA][ARRAY_UPPERINDEX] = iNewSize-1;
      g_iArrInformation[iArrayIndexA][ARRAY_INDEXES] = iNewSize;

      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Subtract"), 1);
}
#endif

#ifdef ARRAY_FUNC_DEBUG

int g_iArrayIndex;
LRESULT CALLBACK DebugDlgProc(HWND hWndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);

// NSIS Function: Display debug window
NSISFUNC(Debug)
{
  NSISARRAY_INIT();

  TCHAR szArrayName[ARRAY_BUFFER_NAME];
  popstring(szArrayName);

  if (*szArrayName)
  {
    int iArrayIndex = GetArrayIndexByName(szArrayName);
    if (iArrayIndex == -1)
      ErrorMsg(hWndParent, MSG_NOT_EXISTS, TEXT("Debug"), 0);
    else
    {
      DialogBox(g_hInstance, MAKEINTRESOURCE(IDD_DIALOG), hWndParent, (DLGPROC)DebugDlgProc);
      NoErrorMsg();
    }
  }
  else
    ErrorMsg(hWndParent, MSG_ERROR, TEXT("Debug"), 0);
}

void LoadArrayDebugInfo(int iArrayIndex, HWND hWndDlg)
{
  HWND hCtl;
  int i, iBytesUsed = 0;
  TCHAR szData[BUFFER_INFO];

  for (i=0; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
    iBytesUsed += my_strlen(g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);

  wsprintf(szData, TEXT("Array #%i: %s"), iArrayIndex+1, g_szArrNames[g_iArrayIndex]);
  SetWindowText(GetDlgItem(hWndDlg, IDC_HEADING), szData);

  hCtl = GetDlgItem(hWndDlg, IDC_LIST);
  SendMessage(hCtl, LB_RESETCONTENT, 0, 0);
  SendMessage(hCtl, LB_INITSTORAGE, g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX], iBytesUsed + g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]*3);

  for (i=0; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
  {
    wsprintf(szData, TEXT("%i"), i);
    TCHAR* pszAdd = (TCHAR*)MALLOC(
      my_strlen(szData, BUFFER_INFO) +
      my_strlen(g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]) + 3);
    wsprintf(pszAdd, TEXT(" %i - %s"), i, g_szArrData[iArrayIndex][i]);
    SendMessage(hCtl, LB_ADDSTRING, 0, (LPARAM)pszAdd);
    FREE(pszAdd);
  }

  wsprintf(szData,
          TEXT(" Indexes used:\r\n  %i / %i\r\n Buffer lengths:\r\n  %i B\r\n Memory used:\r\n  %i B\r\n Memory allocated:\r\n  %i B\r\n\r\n Index reallocation:\r\n  %i\r\n Buffer reallocation:\r\n  %i B"),
          g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]+1,
          g_iArrInformation[iArrayIndex][ARRAY_INDEXES],
          g_iArrInformation[iArrayIndex][ARRAY_BUFFER],
          iBytesUsed,
          g_iArrInformation[iArrayIndex][ARRAY_INDEXES]*g_iArrInformation[iArrayIndex][ARRAY_BUFFER],
          g_iArrInformation[iArrayIndex][ARRAY_REDIM_INDEXES],
          g_iArrInformation[iArrayIndex][ARRAY_REDIM_BUFFERS]);
  SetWindowText(GetDlgItem(hWndDlg, IDC_STATS), szData);

  wsprintf(szData, TEXT(" Arrays in use:\r\n  %i / %i"), g_iArrayCount, (int)ARRAY_MAX_COUNT);
  SetWindowText(GetDlgItem(hWndDlg, IDC_INDEXES), szData);

  hCtl = GetDlgItem(hWndDlg, IDC_NEXT);
  if (iArrayIndex+1 < g_iArrayCount)
    EnableWindow(hCtl, TRUE);
  else
    EnableWindow(hCtl, FALSE);

  hCtl = GetDlgItem(hWndDlg, IDC_PREV);
  if (iArrayIndex-1 >= 0)
    EnableWindow(hCtl, TRUE);
  else
    EnableWindow(hCtl, FALSE);
}

LRESULT CALLBACK DebugDlgProc(HWND hWndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  switch (uMsg)
  {
  case WM_INITDIALOG:
    LoadArrayDebugInfo(g_iArrayIndex, hWndDlg);
  break;
  case WM_COMMAND:
    if (LOWORD(wParam) == IDOK)
      EndDialog(hWndDlg, 0);
    else if (LOWORD(wParam) == IDC_PREV)
    {
      g_iArrayIndex--;
      LoadArrayDebugInfo(g_iArrayIndex, hWndDlg);
      RedrawWindow(hWndDlg, 0, 0, 0);
    }
    else if (LOWORD(wParam) == IDC_NEXT)
    {
      g_iArrayIndex++;
      LoadArrayDebugInfo(g_iArrayIndex, hWndDlg);
      RedrawWindow(hWndDlg, 0, 0, 0);
    }
  break;
  case WM_CLOSE:
    EndDialog(hWndDlg, 0);
  }

  return FALSE;
}

#endif

BOOL ReDimArray(int iArrayIndex, int iIndexes, int iBufferLen)
{
  int i;
  TCHAR **szArrTemp;

  if (iBufferLen == 0)
    return TRUE;

  if (
      ((g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] + iIndexes) >= g_iArrInformation[iArrayIndex][ARRAY_INDEXES])
       ||
      (iBufferLen > g_iArrInformation[iArrayIndex][ARRAY_BUFFER])
     )
  {
    if (iBufferLen > g_iArrInformation[iArrayIndex][ARRAY_BUFFER])
      iBufferLen += g_iArrInformation[iArrayIndex][ARRAY_REDIM_BUFFERS];
    else
      iBufferLen = g_iArrInformation[iArrayIndex][ARRAY_BUFFER];

    if ((g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX] + iIndexes) >= g_iArrInformation[iArrayIndex][ARRAY_INDEXES])
      iIndexes = g_iArrInformation[iArrayIndex][ARRAY_INDEXES] + g_iArrInformation[iArrayIndex][ARRAY_REDIM_INDEXES];
    else
      iIndexes = g_iArrInformation[iArrayIndex][ARRAY_INDEXES];

    szArrTemp = (TCHAR **)MALLOC(sizeof(TCHAR *) * iIndexes);
    if (szArrTemp == NULL)
      return FALSE;

    for (i=0; i<iIndexes; i++)
    {
      szArrTemp[i] = (TCHAR *)MALLOC(sizeof(TCHAR) * iBufferLen);
      if (szArrTemp[i] == NULL)
      {
        FREE(szArrTemp);
        return FALSE;
      }
    }

    for (i=0; i<=g_iArrInformation[iArrayIndex][ARRAY_UPPERINDEX]; i++)
      my_strcpy(szArrTemp[i], iBufferLen, g_szArrData[iArrayIndex][i], g_iArrInformation[iArrayIndex][ARRAY_BUFFER]);

    for (i=0; i<g_iArrInformation[iArrayIndex][ARRAY_INDEXES]; i++)
      FREE(g_szArrData[iArrayIndex][i]);
    FREE(g_szArrData[iArrayIndex]);

    g_szArrData[iArrayIndex] = szArrTemp;
    g_iArrInformation[iArrayIndex][ARRAY_INDEXES] = iIndexes;
    g_iArrInformation[iArrayIndex][ARRAY_BUFFER]  = iBufferLen;
  }
  return TRUE;
}

// Function: Clears stack items up to found "/END"
void ClearStackToEnd()
{
  TCHAR szStackItem[8];
  int i = 0;
  while ((popstring(szStackItem) == 0) && (lstrcmpi(szStackItem, TEXT("/END")) != 0))
    i++;
}

// Function: Finds array index for array name
int GetArrayIndexByName(TCHAR *szArrayName)
{
  for (int i=0; i<g_iArrayCount; i++)
  {
    if (lstrcmpi(g_szArrNames[i], szArrayName) == 0)
      return i;
  }
  return -1;
}

// Function: Search for string inside string
BOOL my_strstr(TCHAR *s1, int len_s1, TCHAR *s2, int len_s2, BOOL bCaseSensitive)
{
  int i = 0, j = 0;
  for (;;)
  {
    if (s2[j] == '\0')
      return TRUE;
    if (s1[i] == '\0')
      return FALSE;
    if (!bCaseSensitive ? LOWORD(CharLower((TCHAR *)s1[i])) == LOWORD(CharLower((TCHAR *)s2[j])) : s1[i] == s2[j])
    {
      i++;
      j++;
    }
    else
    {
      i = i - j + 1;
      j = 0;
    }
  }
  return FALSE;
}

// Function: Copy one string to another with overflow protection.
void my_strcpy(TCHAR *str1, int len1, TCHAR *str2, int len2)
{
  for (int i=0; i<len1; i++)
  {
    if ((i == len1-1) || (i == len2-1) || (str2[i] == '\0'))
    {
      str1[i] = '\0';
      return;
    }
    str1[i] = str2[i];
  }
}

// Function: Get TCHARacter count in TCHARacter array
int my_strlen(TCHAR *str, int len)
{
  for (int i=0; i<len; i++)
    if (str[i] == '\0')
      return i+1;
  return 1;
}

// Function: Converts TCHAR to int
/*int my_atoi(TCHAR *p)
{
  int n=0, f=0;

  for(;;p++) {
    switch(*p) {
    case ' ':
    case '\t':
      continue;
    case '-':
      f++;
    case '+':
      p++;
    }
    break;
  }
  while(*p >= '0' && *p <= '9')
    n = n*10 + *p++ - '0';
  return(f? -n: n);
}*/

BOOL WINAPI DllMain(HANDLE hInst, ULONG ul_reason_for_call, LPVOID lpReserved)
{
  g_hInstance=(HINSTANCE)hInst;
  return TRUE;
}
