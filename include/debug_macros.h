#pragma once

#define CHECK_TRUE(x)                                                      \
  {                                                                        \
    if (!(x)) {                                                            \
      std::cerr << __FILE__ << "(" << __LINE__ << "): failed check " << #x \
                << "\n";                                                   \
      exit(0);                                                             \
    }                                                                      \
  }
