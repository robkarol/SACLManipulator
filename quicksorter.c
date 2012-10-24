/*
 * Robert Karol
 * CS 11 C track
 * Lab 6
 *
 * This function takes in a list of arguments, and sorts them using the
 * quicksort method. The quicksort method sets aside the first element in
 * the list, and divides the rest of the list up into two lists, a list
 * of elements lower than the first list, and a list of elements greater than
 * or equal to the first element. Then it recursively calls this procedure
 * on each of the low and high lists, and appends together the low list with
 * the first element, and high list. This function runs in O(nlog(n)) time.
 */
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "linked_list.h"
#include "memcheck.h"

/* These are the function prototypes. */
node *quicksort(node *list);
int usage(void);

int main(int argc, char *argv[]) {
  /* Type declarations. */
  int i;
  int q = 1;
  int quiet = 0;
  node *current;
  node *result;
  node *previous = NULL;
  /* If there are no arguments, display the usage message. */
  if(q == argc) {
    usage();
  }
  /* Check through each argument in argv[]. */
  for(i = 1; i < argc; i++) {
    /*
     * If the argument is -q, increment q to show there is one less
     * integer argument and set quiet to 1 so that the program does not
     * output the list.
     */
    if(strcmp(argv[i], "-q") == 0) {
      quiet = 1;
      q++;
    }
    /* If there are no integer arguments, display the usage message. */
    if(q == argc) {
      usage();
    }
    /* If it is an integer, place it into the current linked list. */
    else {
      current = create_node(atoi(argv[i]), previous);
      previous = current;
    }
  }
  /*
   * Sort the list using the quicksort function, and set result to
   * be a pointer to the new sorted list.
   */
  result = quicksort(current);
  /* Assert that the list is sorted. */
  assert(is_sorted(result));
  /* Print the sorted list if -q wasn't found in the command arguments. */
  if(quiet == 0) {
    print_list(result);
  }
  /* Free the allocated memory. */
  free_list(current);
  free_list(result);
  /* Print memory leaks. */
  print_memory_leaks();
  return 0;
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



/*
 * This is the usage statement. It prints an error telling the user how
 * to use the program.
 */
int usage(void) {
  fprintf(stderr, "This function takes in a list of integer arguments"
                  " and prints a sorted copy of the list. The argument"
                  " -q supresses the ouput of the function.");
  exit(1);
}
