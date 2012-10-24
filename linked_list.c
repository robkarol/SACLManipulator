/*
 * FILE: linked_list.c
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "memcheck.h"
#include "linked_list.h"


/*
 * create_node:
 *     Create a single node and link it to the node called 'n'.
 */

node *create_node(int data, node *n) {
  node *result = (node *)malloc(sizeof(node));

  if(result == NULL) {
    fprintf(stderr, "Fatal error: out of memory. "
                    "Terminating program.\n");
    exit(1);
  }

  result->data = data;  /* Fill in the new node with the given value. */
  result->next = n;

  return result;
}


/*
 * free_list:
 *     Free all the nodes of a linked list.
 */

void free_list(node *list) {
  node *n;     /* a single node of the list */

  while (list != NULL) {
    n = list;
    list = list->next;
    free(n);
  }
}


/*
 * copy_list:
 *     Return a copy of a list.
 */

node *copy_list(node *list) {
  node *new_list;
  if (list == NULL) {
    return list;
  }
  else {
    new_list = create_node(list->data,copy_list(list->next));
    return new_list;
  }
}


/*
 * append_lists:
 *     Return a list which is a copy of the concatenation of the two
 *     input lists.
 */

node *append_lists(node *list1, node *list2) {
  node *item;

  if(list1 == NULL) {
    return copy_list(list2);
  }
  else if (list2 == NULL) {
    return copy_list(list1);
  }
  else {
    node *new_list1 = copy_list(list1);
    node *new_list2 = copy_list(list2);

    for (item = new_list1; item != NULL; item = item->next) {
      if (item->next == NULL)  /* We're on the last node. */
      {
        item->next = new_list2;
        break;
      }
    }
    return new_list1;
  }
}


/*
 * Make a reversed copy of a list.
 */

node *reverse_list(node *list) {
  node *item;
  node *reversed_list = NULL;

  for(item = list; item != NULL; item = item->next) {
    reversed_list = create_node(item->data, reversed_list);
  }

  return reversed_list;
}


/*
 * Print the elements of a list.
 */

void print_list(node *list) {
  node *item;

  for (item = list; item != NULL; item = item->next) {
    printf("%d\n", item->data);
  }
}

/*
 * Return 1 if a list is sorted, otherwise 0.
 * Useful for debugging.
 */

int is_sorted(node *list) {
  node *item;

  /* An empty list is sorted by definition. */
  if (list == NULL)
    return 1;

  for (item = list; item->next != NULL; item = item->next) {
    if (item->data > item->next->data)  /* wrong order */
      return 0;
  }

  return 1;  /* The list is sorted. */
}

/*
 * This function takes in a pointer to a list, and outputs a pointer
 * to a new list containing the elements of the input list sorted.
 */
node *quicksort(node *list) {
  /* Type declarations. */
  node *result;
  node *temp_low;
  node *temp_high;
  node *first_append;
  node *low = NULL;
  node *high = NULL;
  node *first_element = create_node(list->data, NULL);
  /* If the first element is null, return the null list. */
  if(first_element == NULL) {
    /* Free allocated memory. */
    free_list(low);
    free_list(high);
    free_list(first_element);
    /* Return input list. */
  }
  /* Move the list pointer one farther along the list. */
  list = list->next;
  /*
   * If the second element of the input is list null, return
   * the first element.
   */
  if(list == NULL) {
    /* Free allocated memory. */
    free_list(low);
    free_list(high);
    return first_element;
  }
  /*
   * As long as there are elements left in the list, sort the elements into
   * a list of integers lower than the first element, and a list
   * of integers higher than or equal to the first element.
   */
    while(list != NULL) {
      /*
       * If the list element is lower than the data in the first element,
       * place it into the low linked list.
       */
      if(first_element->data > list->data)
      {
        low = create_node(list->data, low);
      }
      /*
       * If the list element is not lower than the data in the first
       * element, place it into the high linked list.
       */
      else
      {
        high = create_node(list->data, high);
      }
      /* Move the list pointer to the next element. */
      list = list->next;
    }
    /*
     * If there are no elements in the low list,
     * append the first element to the high list.
     */
    if(low == NULL) {
      /* Create a temporary list for the output of quicksort(high). */
      temp_high = quicksort(high);
      /* Free the high list. */
      free_list(high);
      /* Append the first element to the sorted high list. */
      result = append_lists(first_element, temp_high);
      /* Free the sorted high list. */
      free_list(temp_high);
    }
    else if(high == NULL) {
      /* Create a temporary list for the output of quicksort(low). */
      temp_low = quicksort(low);
      /* Free the low list. */
      free_list(low);
      /* Append the sorted low list to the first element. */
      result = append_lists(temp_low, first_element);
      /* Free the sorted low list. */
      free_list(temp_low);
    }
    else
    {
      /* Create a temporary list for the output of quicksort(low). */
      temp_low = quicksort(low);
      /* Free the low list. */
      free_list(low);
      /* Create a temporary list for the output of quicksort(high). */
      temp_high = quicksort(high);
      /* Free the high list. */
      free_list(high);
      /* Append the sorted low list to the first element. */
      first_append = append_lists(temp_low, first_element);
      /* Append the new list to the sorted high list. */
      result = append_lists(first_append, temp_high);
      /* Free allocated memory. */
      free_list(first_append);
      free_list(temp_low);
      free_list(temp_high);
    }
    /* Free allocated memory. */
    free_list(first_element);
    /* Return the sorted list. */
  return result;
}
