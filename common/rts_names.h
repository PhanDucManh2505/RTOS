/*
 * rts_names.h - node names and service names.
 *
 * A QNX server registers a name with name_attach(). A client finds it
 * with name_open(). To reach a server on another machine the client
 * puts the node name in front of the service name; the MsgSend() call
 * itself does not change. That is why moving a process to a different
 * VM only changes a name here, never any code.
 *
 * The three node names are read from the environment so the same
 * binaries run on any set of machines without rebuilding:
 *      RTS_NODE_CENTRAL   default "vm1"
 *      RTS_NODE_INTER     default "vm2"
 *      RTS_NODE_RAIL      default "vm3"
 */
#ifndef RTS_NAMES_H
#define RTS_NAMES_H

#include <stddef.h>

#define SVC_CENTRAL "rts_central"
#define SVC_RAILWAY "rts_railway"

const char *rts_node_central(void);
const char *rts_node_inter(void);
const char *rts_node_rail(void);

/* "rts_i3" : the command channel of intersection 3 */
void rts_svc_inter(char *buf, size_t n, int id);
/* "rts_i3_evt" : the high priority event channel of intersection 3 */
void rts_svc_inter_evt(char *buf, size_t n, int id);

/* The hostname of the machine this process is running on. */
const char *rts_hostname(void);

/*
 * Build the path a client should open.
 * Local  : "rts_central"
 * Remote : "/net/vm1/dev/name/local/rts_central"
 */
void rts_service_path(char *buf, size_t n, const char *node, const char *svc);

#endif /* RTS_NAMES_H */
