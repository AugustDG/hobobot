#pragma once

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define CREATION_CHECK(x)                                                                                              \
  do {                                                                                                                 \
    if ((x) == nullptr) {                                                                                              \
      Serial.printf("ERROR: allocation failed for \"%s\"\n"                                                            \
                    "  in %s:%d\n"                                                                                     \
                    "  function: %s\n",                                                                                \
                    #x,       /* the expression you passed */                                                          \
                    __FILE__, /* current file name */                                                                  \
                    __LINE__, /* current line number */                                                                \
                    __func__  /* current function name */                                                              \
      );                                                                                                               \
      Serial.flush(); /* ensure the message is sent */                                                                 \
      abort();        /* or while(true); to hang, or exit(EXIT_FAILURE); */                                            \
    }                                                                                                                  \
  } while (0)