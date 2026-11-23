/* C ABI of the write_text skill (libskillarm_write.so). JSON in, JSON out (UTF-8).
 *
 *   char* out = NULL;
 *   int ok = sa_write_plan("{\"text\":\"안녕\",\"size_mm\":25,\"paper\":{...},\"pencil\":{...}}", &out);
 *   ... parse out: {"ok":true,"plan_id":"wp_...","feasible":true,"preview_png":"/abs/...","est_duration_s":..,"issues":[..]}
 *   sa_write_free(out);
 *   sa_write_execute("{\"plan_id\":\"wp_...\",\"mode\":\"sim\"}", &out);
 *
 * Return value: 1 when the reply has "ok": true, 0 otherwise (the reply then carries "error"). Thread-safe (calls are
 * serialised inside). Real execution ("mode":"real") is refused unless SKILLARM_WRITE_REAL=1 is set in the
 * environment of the calling process and the request carries "confirm": "<plan_id>".
 */
#ifndef SKILLARM_WRITE_TEXT_H
#define SKILLARM_WRITE_TEXT_H
#ifdef __cplusplus
extern "C" {
#endif

int sa_write_plan(const char* request_json, char** reply_json);
int sa_write_execute(const char* request_json, char** reply_json);
int sa_write_get_plan(const char* request_json, char** reply_json);
void sa_write_free(char* p);

#ifdef __cplusplus
}
#endif
#endif
