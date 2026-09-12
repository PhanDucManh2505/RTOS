#include "rts_names.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *env_or(const char *key, const char *fallback)
{
    const char *v = getenv(key);
    /* An empty variable counts as "not set". */
    if (v != NULL && v[0] != '\0') {
        return v;
    }
    return fallback;
}

const char *rts_node_central(void) { return env_or("RTS_NODE_CENTRAL", "vm1"); }
const char *rts_node_inter(void)   { return env_or("RTS_NODE_INTER",   "vm2"); }
const char *rts_node_rail(void)    { return env_or("RTS_NODE_RAIL",    "vm3"); }

void rts_svc_inter(char *buf, size_t n, int id)
{
    snprintf(buf, n, "rts_i%d", id);
}

void rts_svc_inter_evt(char *buf, size_t n, int id)
{
    snprintf(buf, n, "rts_i%d_evt", id);
}

const char *rts_hostname(void)
{
    static char host[128];
    static int  done = 0;

    if (!done) {
        memset(host, 0, sizeof(host));
        if (gethostname(host, sizeof(host) - 1) != 0) {
            strcpy(host, "unknown");
        }
        done = 1;
    }
    return host;
}

void rts_service_path(char *buf, size_t n, const char *node, const char *svc)
{
    /*
     * If the server is on this machine we open the plain name. That
     * keeps the system working even when Qnet is not running yet.
     */
    if (node == NULL || node[0] == '\0' || strcmp(node, rts_hostname()) == 0) {
        snprintf(buf, n, "%s", svc);
    } else {
        snprintf(buf, n, "/net/%s/dev/name/local/%s", node, svc);
    }
}
