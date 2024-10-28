
#ifndef YASMIN_LOGS_HPP
#define YASMIN_LOGS_HPP

#define YASMIN_LOG_ERROR(text, ...)                                            \
  fprintf(stderr, "[ERROR] " text "\n", ##__VA_ARGS__)
#define YASMIN_LOG_WARN(text, ...)                                             \
  fprintf(stderr, "[WARN] " text "\n", ##__VA_ARGS__)
#define YASMIN_LOG_INFO(text, ...)                                             \
  fprintf(stderr, "[INFO] " text "\n", ##__VA_ARGS__)
#define YASMIN_LOG_DEBUG(text, ...)                                            \
  fprintf(stderr, "[DEBUG] " text "\n", ##__VA_ARGS__)

#endif