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

//=============
//  Includes
//=============

#include "lists.h"

//------------------------------------------------------------------------------

int CreateIntList(IntList **list, int type)
{
	*list = (IntList*)malloc(sizeof(IntList));
	
	if ( *list == NULL)
	{
		printf("CreateIntList: Cannot create list (out of memory?)...\n");
		return -1;
	}
	
	(*list)->head = NULL;
	(*list)->tail = NULL;
	(*list)->type = type;
	(*list)->count = 0;
	
	return 0;
}

//=============================================================================

int ClearIntList(IntList* list)
{
	if (list == NULL)
	{
		printf("ClearIntList: List is not initialised...\n");
		return -1;
	}
	
	while (list->count > 0)
	{
		PopInt(list);
	}
	
	return 0;
}

//=============================================================================

int FreeIntList(IntList* list)
{
	int ret = 0;
	
	if (list == NULL)
	{
		printf("FreeIntList: List is not initialised...\n");
		return -1;
	}
	
	if ( (ret = ClearIntList(list)) != 0 ){
		printf("FreeIntList : ClearIntList error\n");
		return -1;
	}  
	
	free(list);
	list = NULL;
	
return 0;
}

//=============================================================================

int PopInt(IntList* list)
{
  IntListElement* temp;
  int value;

  if (list == NULL)
  {
    printf("List is not initialised...\n");
    return -1;
  }

  if(list->count == 0)
  {
    printf("Pop: Cannot pop, list is empty!\n");
    return -1;
  }

  //get the data from the head element.
  value = list->head->value;

  //get the adress to the head element.
  temp = list->head;

  //move the head pointer of the list.
  list->head = list->head->next;

  //give back the element to the mem_space.
  free(temp);

  //decrement the list->count counter.
  list->count--;

  return value;
}

//=============================================================================

int PushInt(IntList* list, int value)
{
	IntListElement* before;
	IntListElement* after;
	
	//temporary pointer to a element.
	IntListElement* temp;
	if (list == NULL)
	{
		printf("PushInt: List is not initialised...\n");
		return -1;
	}
	
	temp = (IntListElement*) malloc(sizeof(IntListElement));
	if (temp == NULL)
	{
		printf("PushInt: Out of memory - list is full!\n");
		return -1;
	}
	
	temp->value = value;
	
	//check if it's the head element.
	if(list->count == 0)
	{
		//Set the head and tail pointers to the new element
		list->head = temp;
		list->tail = list->head;
		
		//Set the next pointer in the tail element to NULL.
		list->tail->next = NULL;
	}
	else
	{
		//Here is where the sorting is done.
		if (list->type == LIFO)
		{
			//Insert the new element in the head place of the list.
			temp->next = list->head;
			list->head = temp;
		}
		else if (list->type == FIFO)
		{
			//Insert the new element in the tail place of the list.
			temp->next = NULL;
			list->tail->next = temp;
			list->tail = temp;
		}
		else if (list->type == SORTED)
		{
			//First check if new item will be first
			if (list->head->value > value)
			{
				temp->next = list->head;
				list->head = temp;
			}
			//Then check if new item will be last
			else if (list->tail->value <= value)
			{
				temp->next = NULL;
				list->tail->next = temp;
				list->tail = temp;
			}
			//else find correct location for new item
			else
			{
				before = list->head;
				after = list->head->next;
				
				while (after->value <= value)
				{
					before = before->next;
					after = after->next;
				}
				
				before->next = temp;
				temp->next = after;
			}
		}
		else
		{
			printf("Invalid list type!\n");
			return -1;
		}
	}
	
	//increment the list->count counter.
	list->count++;
	
	return 0;
}

//=============================================================================

int IsIntListEmpty(IntList* list)
{
  if (list == NULL)
  {
    printf("IsIntListEmpty: List is not initialised...\n");
    return -1;
  }

  if (list->count == 0)
    return 1;
  else
    return 0;
}

//=============================================================================

int PrintIntList(IntList *list)
{
	int i;
	
	if (list == NULL)
	{
		printf("PrintIntList: List is not initialised...\n");
		return -1;
	}
	
	IntListElement* temp = list->head;
	
	for (i=0;i<list->count;i++)
	{
		printf("(%d) [%d/%d] \n", temp->value, i, list->count);
		temp = temp->next;
	}
	return 0;
}

//=============================================================================

int ChangeIntListType(IntList* list, int type)
{
	int ret = 0;
	
	if (list == NULL) 
	{
		printf("List is not initialised...\n");
		return -1;
	}
	
	
	
	if ( (ret = ClearIntList(list)) == 0 ){
		printf("ChangeIntListType : ClearIntList error");
		return -1;
	}
	
	list->type = type;

	return 0;
}

//==============================================================================

int CreateComponent(Component **comp, int label)
{
	*comp = (Component*)malloc(sizeof(Component));
	
	if (*comp == NULL)
	{
		printf("CreateComponent: Cannot create component (out of memory?)...\n");
		return -1;
	}
	
	(*comp)->label = label;
	(*comp)->color = -1;
	(*comp)->centroidRow = -1;
	(*comp)->centroidCol = -1;
	(*comp)->centroidX = -1;
	(*comp)->centroidY = -1;
	
	IntList *pixelList;
	CreateIntList(&pixelList,2); //FIFO
	(*comp)->pixelList = pixelList;
	
return 0;
}

//==============================================================================

int CreateComponentsList(ComponentsList **list, int type)
{
	*list = (ComponentsList*)malloc(sizeof(ComponentsList));

	if (*list == NULL)
	{
		printf("CreateComponentsList: Cannot create list (out of memory?)...\n");
		return -1;
	}
	
	(*list)->head = NULL;
	(*list)->tail = NULL;
	(*list)->type = type;
	(*list)->count = 0;
	
	return 0;
}

//=============================================================================

int ClearComponentsList(ComponentsList* list)
{
	int ret = 0;
	Component *temp;
	CreateComponent(&temp,0);
	
	if (list == NULL)
	{
		printf("ClearComponentsList: List is not initialised...\n");
		return -1;
	}
	
	
	while (list->count > 0){	
		if ( (ret = PopComponent(list,temp)) != 0) {
			printf("ClearComponentsList, PopComponent error \n");
			return -1;
		}
	}
	return 0;	
}

//=============================================================================

int FreeComponentsList(ComponentsList* list)
{
	int ret;
	
	if (list == NULL)
	{
		printf("FreeComponentsList: List is not initialised...\n");
		return -1;
	}
	
	if ( (ret = ClearComponentsList(list)) == 0){
		printf("FreeComponentsList, ClearComponentList error \n");
		return -1;
	}
		
	free(list);
	list = NULL;
		
	return 0;
}

//=============================================================================

int PopComponent(ComponentsList* list, Component *c)
{
	ComponentsListElement* temp;
	int ret = 0;
	
	if (list == NULL)
	{
		printf("PopComponent:List is not initialised...\n");
		return -1;
	}
	
	if(list->count == 0)
	{
		printf("PopComponent: Cannot pop, list is empty!\n");
		return -1;
	}
	
	//get the data from the head element.
	*c = list->head->component;
	
	//get the adress to the head element.
	temp = list->head;
	
	//move the head pointer of the list.
	list->head = list->head->next;
	
	//give back the element to the mem_space.
	if ( (ret = FreeIntList(temp->component.pixelList)) != 0){
		printf("FreeComponentsList, FreeIntList error \n");
		return -1;
	}
	
	free(temp);
	
	//decrement the list->count counter.
	list->count--;
	
	return 0;
}

//=============================================================================

int PushComponent(ComponentsList* list, Component c)
{
	ComponentsListElement* before;
	ComponentsListElement* after;
	
	//temporary pointer to a element.
	ComponentsListElement* temp;
	if (list == NULL)
	{
		printf("PushComponent: List is not initialised...\n");
		return -1;
	}
	
	temp = (ComponentsListElement*)malloc(sizeof(ComponentsListElement));
	if (temp == NULL)
	{
		printf("PushComponent: Out of memory - list is full!\n");
		return -1;
	}
	
	temp->component = c;
	
	//check if it's the head element.
	if(list->count == 0)
	{
		//Set the head and tail pointers to the new element
		list->head = temp;
		list->tail = list->head;
		
		//Set the next pointer in the tail element to NULL.
		list->tail->next = NULL;
	}
	else
	{
		//Here is where the sorting is done.
		if (list->type == LIFO)
		{
			//Insert the new element in the head place of the list.
			temp->next = list->head;
			list->head = temp;
		}
		else if (list->type == FIFO)
		{
			//Insert the new element in the tail place of the list.
			temp->next = NULL;
			list->tail->next = temp;
			list->tail = temp;
		}
		else if (list->type == SORTED)
		{
			//First check if new item will be first
			if (list->head->component.label > c.label)
			{
				temp->next = list->head;
				list->head = temp;
			}
			//Then check if new item will be last
			else if (list->tail->component.label <= c.label)
			{
				temp->next = NULL;
				list->tail->next = temp;
				list->tail = temp;
			}
			//else find correct location for new item
			else
			{
				before = list->head;
				after = list->head->next;
				
				while (after->component.label <= c.label)
				{
					before = before->next;
					after = after->next;
				}
				
				before->next = temp;
				temp->next = after;
			}
		}
		else
		{
			printf("PushComponent: Invalid list type!\n");
			return -1;
		}
	}
	
	//increment the list->count counter.
	list->count++;
	
	return 0;
}

//=============================================================================

int IsComponentsListEmpty(ComponentsList* list)
{
  if (list == NULL)
  {
    printf("IsComponentsListEmpty: List is not initialised...\n");
    return -1;
  }

  if (list->count == 0)
    return 1;
  else
    return 0;
}

//=============================================================================

int PrintComponentsList(ComponentsList* list)
{
	int i;
	
	printf("List type : %d \n", list->type);
	printf("List element count : %d \n", list->count);
	
	ComponentsListElement* temp = list->head;
	
	for (i=0;i<list->count;i++)
	{  	
		printf("Component label: (%d)\n", temp->component.label);
		printf("Component centroid : X= %d, Y=%d\n",temp->component.centroidRow,temp->component.centroidCol);
		printf("Indexes of the pixels belonging to component: \n");
		PrintIntList(temp->component.pixelList);
		temp = temp->next;
	}
return 0;
}

//=============================================================================

int ChangeComponentsListType(ComponentsList* list, int type)
{
	if (list == NULL)
	{
		printf("List is not initialised...\n");
		return -1;
	}
	
	ClearComponentsList(list);
	list->type = type;
	
	return 0;
}

//=============================================================================

int  AddPixelToComponent(ComponentsList* list, int componentLabel, int pixel)
{

int i = 1;

 //temporary pointer to a element.
 ComponentsListElement* temp;

 temp = list->head;
 
 // Find the specified component
 //printf(" looking for label %d\n",componentLabel);

 while ( i<list->count && (temp->component.label != componentLabel) ){
    //printf("curren component label %d\n",temp->component.label);
    //printf("moving to the next component\n");    
    temp = temp->next;
    i++;
 }
 
 if (temp->component.label != componentLabel) {
    printf("AddPixelToComponent: component with label %d not found, error in pixel [%d,%d](%d) insertion \n",componentLabel,pixel/320,pixel%320,pixel);
    //getchar();
    //exit(-1);
    return -1;
 }

 else {

    //printf("Found pointer to component with label %d\n",temp->component.label);
    // push the pixel in the intList of the component
    PushInt(temp->component.pixelList,pixel); 
    //printf("Int pushed in IntList\n");
 
 }

return 0;
}

