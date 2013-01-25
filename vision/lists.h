//==========================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Simple list management implementation. The lists are not 
//                  generic, there is a lot of code duplication. Lots of room 
//					for refactoring and optimization in here.
//
//  Description:    Provides a C implementation to simple list data structures.
//
//  Author:         Enrico Di Lello
//  Author:  	    and many small changes by Giovanni Micheli
//
//  
//--------------------------------------------------------------------------
//  FoxVision: Vision system for mobile robots
//  Copyright (c) 2009 Enrico Di Lello
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
//  or obtain a copy from the GNU website at http://www.gnu.org/
//
//==========================================================================

#ifndef lists_h_DEFINED
#define lists_h_DEFINED

//================
//   Includes
//================

#include <stdlib.h>
#include <stdio.h>


//===============
//   Defines
//===============

//Types of lists

#define SORTED 0
#define LIFO   1
#define FIFO   2

#define LIST_TYPE(x) \
 (((x)==SORTED) ? "SORTED" :\
  ((x)==LIFO)   ? "LIFO"   :\
  ((x)==FIFO)   ? "FIFO"   : "UNKNOWN")


//Error Tracing

#define TraceErrors 1


//====================
//     Structures
//====================

// List of int values, used for memorizing the index of the pixels belonging to
// the components
 
typedef struct IntListElementType{

int value;
struct IntListElementType* next;

}IntListElement;

// Int list

typedef struct
{
  IntListElement *head;      //pointer to first element in list
  IntListElement *tail;      //pointer to last element in list
  int count;                 //number of elements in list
  int type;                  //type of list being used (SORTED, LIFO, FIFO)
}IntList;

// List of connected components, used for memorizing the labels and the pixel 
// indexes of all the blobs in the image

// Contains a list of the pixel index belonging to 
// the component,  the centroid value and the corrisponding label

typedef struct 
{

 IntList* pixelList;            // IntList of pixel belongings to the component
 int label;			// label of the current connected component
 int color;			// codice colore identificato nell'area relativa alla componente	
 int centroidRow;               // Row pixel coordinate of blob centroid  
 int centroidCol;		// Column pixel coordinate of blob centroid
 float centroidX;               // X coordinate in the of the blob centroid in the floor reference frame 
 float centroidY;		// Y coordinate in the of the blob centroid in the floor reference frame
 float centroid_distance;         // estimated distance
 float centroid_angle;             // estimated deviation angle
}Component;

// Components list element

typedef struct ComponentsListElementType
{
 Component component;
 struct ComponentsListElementType* next;
}ComponentsListElement;

// Components list

typedef struct
{
  ComponentsListElement *head;    //pointer to first element in list
  ComponentsListElement *tail;    //pointer to last element in list
  int count;                      //number of elements in list
  int type;                       //type of list being used (SORTED, LIFO, FIFO)
}ComponentsList;


//==============================
//    Function prototypes
//==============================

//===========
/// IntList 
//===========

//--------------------------------------------------------------------------
///
/// Create an IntList.
///
/// \param  list          IntList double pointer
/// \param  type          list type: can be LIFO,FIFO,SORTED
///
//--------------------------------------------------------------------------

int CreateIntList(IntList **list, int type);            

//--------------------------------------------------------------------------
///
/// Empties a IntList 
///
/// \param  list          The list to clear
///
//--------------------------------------------------------------------------

int ClearIntList(IntList* list);                       

//------------------------------------------------------------------------------
///
/// Realeases a IntList and all its associated memory
///
/// \param  list          The list to release
///
//------------------------------------------------------------------------------

int FreeIntList(IntList* list);                       

//------------------------------------------------------------------------------
///
/// Pop a IntList element 
///
/// \param  list          The list to pop
///
//------------------------------------------------------------------------------

int  PopInt(IntList* list);

//--------------------------------------------------------------------------
///
/// Push a int value into a IntList 
///
/// \param  list          The list to push into
/// \param  value         The int value to push
/// 
//--------------------------------------------------------------------------

int PushInt(IntList* list, int value);

//--------------------------------------------------------------------------
///
/// Checks if an IntList is empty
///
/// \param  list          The list to check
/// 
//--------------------------------------------------------------------------

int  IsIntListEmpty(IntList* list);

//--------------------------------------------------------------------------
///
/// Print all elements of an IntList 
///
/// \param  list          The list to print
///
//--------------------------------------------------------------------------

int PrintIntList(IntList* list);

//--------------------------------------------------------------------------
///
/// Change IntList type 
///
/// \param  list          The list to change
///
//--------------------------------------------------------------------------

int ChangeIntListType(IntList* list,int type);

//====================
///  ComponentsList
//====================

//------------------------------------------------------------------------------
///
/// Create a Component with an empty pixelList.
///
/// \param  comp          double pointer to the component to be created
/// \param  label          component label
///
//------------------------------------------------------------------------------

int CreateComponent(Component **comp ,int label);                 

//------------------------------------------------------------------------------
///
/// Create a ComponentsList.
///
/// \param  list          IntList double pointer
/// \param  type          list type: can be LIFO,FIFO,SORTED
///
//------------------------------------------------------------------------------

int CreateComponentsList(ComponentsList **list, int type);         

//------------------------------------------------------------------------------
///
/// Empties a ComponentsList 
///
/// \param  list          The list to clear
///
//------------------------------------------------------------------------------
 
int ClearComponentsList(ComponentsList *list);         

//------------------------------------------------------------------------------
///
/// Realeases a ComponentsList and all its associated memory
///
/// \param  list          The list to release
///
//------------------------------------------------------------------------------

int FreeComponentsList(ComponentsList *list);          

//------------------------------------------------------------------------------
///
/// Pop a ComponentsList element 
///
/// \param  list          The list to pop
/// \param  temp          Pointer to the component to pop 
///
//------------------------------------------------------------------------------

int PopComponent(ComponentsList *list, Component *c);          

//------------------------------------------------------------------------------
///
/// Push a Component structure into a ComponentsList 
///
/// \param  list          The list to push into
/// \param  c             The component value to push
/// 
//------------------------------------------------------------------------------

int PushComponent(ComponentsList* list, Component c); 

//------------------------------------------------------------------------------
///
/// Checks if a ComponentsList is empty
///
/// \param  list          The list to check
/// 
//------------------------------------------------------------------------------

int  IsCompListEmpty(ComponentsList* list);                

//------------------------------------------------------------------------------
///
/// Print all elements of an ComponentsList 
///
/// \param  list          The list to print
///
//------------------------------------------------------------------------------

int PrintComponentsList(ComponentsList* list);         

//------------------------------------------------------------------------------
///
/// Change IntList type 
///
/// \param  list          The list to change
/// \param  type          The list type
///
//------------------------------------------------------------------------------

int ChangeComponentsListType(ComponentsList* list, int type);    

//------------------------------------------------------------------------------
///
/// Add a pixel index to a given component 
///
/// \param  list                The components list 
/// \param  componentLabel      The label ot the component whom the pixel belongs
/// \param  pixel               The pixel index 
///
//------------------------------------------------------------------------------

int  AddPixelToComponent(ComponentsList* list, int componentLabel, int pixel); //add a pixel in the specified component  


//=============================================================================
#endif //#ifndef lists_h_DEFINED
//=============================================================================
