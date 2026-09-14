# ---------------------------------------------------------------------
# EEET2588 Real-Time Systems - Traffic Light Control System
# Team ANK
#
#   make            build every program into bin/
#   make clean      remove bin/ and the object files
#   make DEPLOY_HOST=192.168.56.110 deploy      copy bin/ to one target
#
# The QNX compiler is qcc. -V picks the target variant; run "qcc -V"
# on your machine to list the ones your SDP installed.
# ---------------------------------------------------------------------

QCC      ?= qcc
VARIANT  ?= gcc_ntox86_64

CFLAGS   := -V$(VARIANT) -Wall -O1 -g -Icommon -D_QNX_SOURCE
LDFLAGS  := -V$(VARIANT)
LDLIBS   := -lsocket

BIN      := bin
OBJ      := obj

COMMON_SRC := common/rts_proto.c  common/rts_names.c common/rts_util.c \
              common/rts_log.c    common/rts_safety.c common/rts_color.c
COMMON_OBJ := $(patsubst common/%.c,$(OBJ)/%.o,$(COMMON_SRC))

CORE_OBJ   := $(OBJ)/intersection_core.o

INTERSECTIONS := i1 i2 i3 i4 i5 i6
INTER_BINS    := $(addprefix $(BIN)/intersection_,$(INTERSECTIONS))

ALL_BINS := $(BIN)/central $(BIN)/railway $(BIN)/inter_panel $(INTER_BINS)

.PHONY: all clean deploy dirs

all: dirs $(ALL_BINS)
	@echo ""
	@echo "built:"
	@ls -1 $(BIN)

dirs:
	@mkdir -p $(BIN) $(OBJ)

$(OBJ)/%.o: common/%.c
	$(QCC) $(CFLAGS) -c $< -o $@

$(OBJ)/%.o: src/%.c
	$(QCC) $(CFLAGS) -c $< -o $@

$(BIN)/central: $(OBJ)/central.o $(COMMON_OBJ)
	$(QCC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BIN)/railway: $(OBJ)/railway.o $(COMMON_OBJ)
	$(QCC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BIN)/inter_panel: $(OBJ)/inter_panel.o $(COMMON_OBJ)
	$(QCC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BIN)/intersection_%: $(OBJ)/intersection_%.o $(CORE_OBJ) $(COMMON_OBJ)
	$(QCC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	rm -rf $(BIN) $(OBJ)

# Copy the binaries and the run scripts onto one target.
DEPLOY_HOST ?= 192.168.56.110
DEPLOY_DIR  ?= /tmp/manh
deploy: all
	ssh root@$(DEPLOY_HOST) "mkdir -p $(DEPLOY_DIR) /fs/rts"
	scp $(BIN)/* scripts/*.sh root@$(DEPLOY_HOST):$(DEPLOY_DIR)/
	ssh root@$(DEPLOY_HOST) "chmod +x $(DEPLOY_DIR)/*"
	@echo "deployed to $(DEPLOY_HOST):$(DEPLOY_DIR)"
