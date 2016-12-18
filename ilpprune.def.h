











/* ---------------- method closures -------------- */
#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY

    struct Closure_Master::updateOFUB_3_closure : public SDAG::Closure {
      double ofub;
      int size;
      double *solution;

      CkMarshallMsg* _impl_marshall;
      char* _impl_buf_in;
      int _impl_buf_size;

      updateOFUB_3_closure() {
        init();
        _impl_marshall = 0;
        _impl_buf_in = 0;
        _impl_buf_size = 0;
      }
      updateOFUB_3_closure(CkMigrateMessage*) {
        init();
        _impl_marshall = 0;
        _impl_buf_in = 0;
        _impl_buf_size = 0;
      }
      double & getP0() { return ofub;}
      int & getP1() { return size;}
      double *& getP2() { return solution;}
      void pup(PUP::er& __p) {
        __p | ofub;
        __p | size;
        packClosure(__p);
        __p | _impl_buf_size;
        bool hasMsg = (_impl_marshall != 0); __p | hasMsg;
        if (hasMsg) CkPupMessage(__p, (void**)&_impl_marshall);
        else PUParray(__p, _impl_buf_in, _impl_buf_size);
        if (__p.isUnpacking()) {
          char *impl_buf = _impl_marshall ? _impl_marshall->msgBuf : _impl_buf_in;
          PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  int size; implP|size;
  int impl_off_solution, impl_cnt_solution; 
  implP|impl_off_solution;
  implP|impl_cnt_solution;
          impl_buf+=CK_ALIGN(implP.size(),16);
          solution = (double *)(impl_buf+impl_off_solution);
        }
      }
      virtual ~updateOFUB_3_closure() {
        if (_impl_marshall) CmiFree(UsrToEnv(_impl_marshall));
      }
      PUPable_decl(SINGLE_ARG(updateOFUB_3_closure));
    };
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY

    struct Closure_Master::done_4_closure : public SDAG::Closure {
      

      done_4_closure() {
        init();
      }
      done_4_closure(CkMigrateMessage*) {
        init();
      }
            void pup(PUP::er& __p) {
        packClosure(__p);
      }
      virtual ~done_4_closure() {
      }
      PUPable_decl(SINGLE_ARG(done_4_closure));
    };
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */


/* ---------------- method closures -------------- */
#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY

    struct Closure_Cache::updateOFUB_2_closure : public SDAG::Closure {
      double ofub;


      updateOFUB_2_closure() {
        init();
      }
      updateOFUB_2_closure(CkMigrateMessage*) {
        init();
      }
      double & getP0() { return ofub;}
      void pup(PUP::er& __p) {
        __p | ofub;
        packClosure(__p);
      }
      virtual ~updateOFUB_2_closure() {
      }
      PUPable_decl(SINGLE_ARG(updateOFUB_2_closure));
    };
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY

    struct Closure_Cache::postConstraint_3_closure : public SDAG::Closure {
      set<constraint_t > constraints;


      postConstraint_3_closure() {
        init();
      }
      postConstraint_3_closure(CkMigrateMessage*) {
        init();
      }
      set<constraint_t > & getP0() { return constraints;}
      void pup(PUP::er& __p) {
        __p | constraints;
        packClosure(__p);
      }
      virtual ~postConstraint_3_closure() {
      }
      PUPable_decl(SINGLE_ARG(postConstraint_3_closure));
    };
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY

    struct Closure_Cache::postFinishedConstraint_4_closure : public SDAG::Closure {
      constraint_set_t constraints;


      postFinishedConstraint_4_closure() {
        init();
      }
      postFinishedConstraint_4_closure(CkMigrateMessage*) {
        init();
      }
      constraint_set_t & getP0() { return constraints;}
      void pup(PUP::er& __p) {
        __p | constraints;
        packClosure(__p);
      }
      virtual ~postFinishedConstraint_4_closure() {
      }
      PUPable_decl(SINGLE_ARG(postFinishedConstraint_4_closure));
    };
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY

    struct Closure_Cache::postDroppedConstraint_5_closure : public SDAG::Closure {
      set<constraint_set_t > constraints;


      postDroppedConstraint_5_closure() {
        init();
      }
      postDroppedConstraint_5_closure(CkMigrateMessage*) {
        init();
      }
      set<constraint_set_t > & getP0() { return constraints;}
      void pup(PUP::er& __p) {
        __p | constraints;
        packClosure(__p);
      }
      virtual ~postDroppedConstraint_5_closure() {
      }
      PUPable_decl(SINGLE_ARG(postDroppedConstraint_5_closure));
    };
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY

    struct Closure_Cache::reportStat_6_closure : public SDAG::Closure {
      

      reportStat_6_closure() {
        init();
      }
      reportStat_6_closure(CkMigrateMessage*) {
        init();
      }
            void pup(PUP::er& __p) {
        packClosure(__p);
      }
      virtual ~reportStat_6_closure() {
      }
      PUPable_decl(SINGLE_ARG(reportStat_6_closure));
    };
#endif /* CK_TEMPLATES_ONLY */


/* ---------------- method closures -------------- */
#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */


/* DEFS: readonly CProxy_Master mainProxy;
 */
extern CProxy_Master mainProxy;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_mainProxy(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|mainProxy;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly CProxy_Cache cacheProxy;
 */
extern CProxy_Cache cacheProxy;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_cacheProxy(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|cacheProxy;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly map_t problem;
 */
extern map_t problem;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_problem(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|problem;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly distance_t dist;
 */
extern distance_t dist;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_dist(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|dist;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly idx_map_t variable_map;
 */
extern idx_map_t variable_map;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_variable_map(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|variable_map;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly double_vec_t objective_vec;
 */
extern double_vec_t objective_vec;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_objective_vec(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|objective_vec;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly double_vec_t col_lb_vec;
 */
extern double_vec_t col_lb_vec;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_col_lb_vec(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|col_lb_vec;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly double_vec_t col_ub_vec;
 */
extern double_vec_t col_ub_vec;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_col_ub_vec(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|col_ub_vec;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly constraint_vec_t base_constraint_vec;
 */
extern constraint_vec_t base_constraint_vec;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_base_constraint_vec(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|base_constraint_vec;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly int n_cols;
 */
extern int n_cols;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_n_cols(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|n_cols;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly int cache_size;
 */
extern int cache_size;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_cache_size(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|cache_size;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: readonly int granularity;
 */
extern int granularity;
#ifndef CK_TEMPLATES_ONLY
extern "C" void __xlater_roPup_granularity(void *_impl_pup_er) {
  PUP::er &_impl_p=*(PUP::er *)_impl_pup_er;
  _impl_p|granularity;
}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: mainchare Master: Chare{
Master(CkArgMsg* impl_msg);
void groupCreated(CkReductionMsg* impl_msg);
void updateOFUB(double ofub, int size, const double *solution);
void done(void);
void getStat(CkReductionMsg* impl_msg);
};
 */
#ifndef CK_TEMPLATES_ONLY
 int CkIndex_Master::__idx=0;
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
/* DEFS: Master(CkArgMsg* impl_msg);
 */

CkChareID CProxy_Master::ckNew(CkArgMsg* impl_msg, int impl_onPE)
{
  CkChareID impl_ret;
  CkCreateChare(CkIndex_Master::__idx, CkIndex_Master::idx_Master_CkArgMsg(), impl_msg, &impl_ret, impl_onPE);
  return impl_ret;
}

void CProxy_Master::ckNew(CkArgMsg* impl_msg, CkChareID* pcid, int impl_onPE)
{
  CkCreateChare(CkIndex_Master::__idx, CkIndex_Master::idx_Master_CkArgMsg(), impl_msg, pcid, impl_onPE);
}

  CProxy_Master::CProxy_Master(CkArgMsg* impl_msg, int impl_onPE)
{
  CkChareID impl_ret;
  CkCreateChare(CkIndex_Master::__idx, CkIndex_Master::idx_Master_CkArgMsg(), impl_msg, &impl_ret, impl_onPE);
  ckSetChareID(impl_ret);
}

// Entry point registration function

int CkIndex_Master::reg_Master_CkArgMsg() {
  int epidx = CkRegisterEp("Master(CkArgMsg* impl_msg)",
      _call_Master_CkArgMsg, CMessage_CkArgMsg::__idx, __idx, 0);
  CkRegisterMessagePupFn(epidx, (CkMessagePupFn)CkArgMsg::ckDebugPup);
  return epidx;
}


void CkIndex_Master::_call_Master_CkArgMsg(void* impl_msg, void* impl_obj_void)
{
  Master* impl_obj = static_cast<Master *>(impl_obj_void);
  new (impl_obj) Master((CkArgMsg*)impl_msg);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void groupCreated(CkReductionMsg* impl_msg);
 */

void CProxy_Master::groupCreated(CkReductionMsg* impl_msg)
{
  ckCheck();
  if (ckIsDelegated()) {
    int destPE=CkChareMsgPrep(CkIndex_Master::idx_groupCreated_CkReductionMsg(), impl_msg, &ckGetChareID());
    if (destPE!=-1) ckDelegatedTo()->ChareSend(ckDelegatedPtr(),CkIndex_Master::idx_groupCreated_CkReductionMsg(), impl_msg, &ckGetChareID(),destPE);
  }
  else CkSendMsg(CkIndex_Master::idx_groupCreated_CkReductionMsg(), impl_msg, &ckGetChareID(),0);
}

void CkIndex_Master::_call_redn_wrapper_groupCreated_CkReductionMsg(void* impl_msg, void* impl_obj_void)
{
  Master* impl_obj = static_cast<Master*> (impl_obj_void);
  char* impl_buf = (char*)((CkReductionMsg*)impl_msg)->getData();
  impl_obj->groupCreated((CkReductionMsg*)impl_msg);
  delete (CkReductionMsg*)impl_msg;
}


// Entry point registration function

int CkIndex_Master::reg_groupCreated_CkReductionMsg() {
  int epidx = CkRegisterEp("groupCreated(CkReductionMsg* impl_msg)",
      _call_groupCreated_CkReductionMsg, CMessage_CkReductionMsg::__idx, __idx, 0);
  CkRegisterMessagePupFn(epidx, (CkMessagePupFn)CkReductionMsg::ckDebugPup);
  return epidx;
}


// Redn wrapper registration function

int CkIndex_Master::reg_redn_wrapper_groupCreated_CkReductionMsg() {
  return CkRegisterEp("redn_wrapper_groupCreated(CkReductionMsg *impl_msg)",
      _call_redn_wrapper_groupCreated_CkReductionMsg, CMessage_CkReductionMsg::__idx, __idx, 0);
}


void CkIndex_Master::_call_groupCreated_CkReductionMsg(void* impl_msg, void* impl_obj_void)
{
  Master* impl_obj = static_cast<Master *>(impl_obj_void);
  impl_obj->groupCreated((CkReductionMsg*)impl_msg);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void updateOFUB(double ofub, int size, const double *solution);
 */

void CProxy_Master::updateOFUB(double ofub, int size, const double *solution, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: double ofub, int size, const double *solution
  int impl_off=0;
  int impl_arrstart=0;
  int impl_off_solution, impl_cnt_solution;
  impl_off_solution=impl_off=CK_ALIGN(impl_off,sizeof(double));
  impl_off+=(impl_cnt_solution=sizeof(double)*(size));
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|ofub;
    implP|size;
    implP|impl_off_solution;
    implP|impl_cnt_solution;
    impl_arrstart=CK_ALIGN(implP.size(),16);
    impl_off+=impl_arrstart;
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|ofub;
    implP|size;
    implP|impl_off_solution;
    implP|impl_cnt_solution;
  }
  char *impl_buf=impl_msg->msgBuf+impl_arrstart;
  memcpy(impl_buf+impl_off_solution,solution,impl_cnt_solution);
  if (ckIsDelegated()) {
    int destPE=CkChareMsgPrep(CkIndex_Master::idx_updateOFUB_marshall3(), impl_msg, &ckGetChareID());
    if (destPE!=-1) ckDelegatedTo()->ChareSend(ckDelegatedPtr(),CkIndex_Master::idx_updateOFUB_marshall3(), impl_msg, &ckGetChareID(),destPE);
  }
  else CkSendMsg(CkIndex_Master::idx_updateOFUB_marshall3(), impl_msg, &ckGetChareID(),0+CK_MSG_EXPEDITED);
}

// Entry point registration function

int CkIndex_Master::reg_updateOFUB_marshall3() {
  int epidx = CkRegisterEp("updateOFUB(double ofub, int size, const double *solution)",
      _call_updateOFUB_marshall3, CkMarshallMsg::__idx, __idx, 0+CK_EP_NOKEEP);
  CkRegisterMarshallUnpackFn(epidx, _callmarshall_updateOFUB_marshall3);
  CkRegisterMessagePupFn(epidx, _marshallmessagepup_updateOFUB_marshall3);

  return epidx;
}


void CkIndex_Master::_call_updateOFUB_marshall3(void* impl_msg, void* impl_obj_void)
{
  Master* impl_obj = static_cast<Master *>(impl_obj_void);
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double ofub, int size, const double *solution*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  int size; implP|size;
  int impl_off_solution, impl_cnt_solution; 
  implP|impl_off_solution;
  implP|impl_cnt_solution;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  double *solution=(double *)(impl_buf+impl_off_solution);
  impl_obj->updateOFUB(ofub, size, solution);
}

int CkIndex_Master::_callmarshall_updateOFUB_marshall3(char* impl_buf, void* impl_obj_void) {
  Master* impl_obj = static_cast< Master *>(impl_obj_void);
  /*Unmarshall pup'd fields: double ofub, int size, const double *solution*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  int size; implP|size;
  int impl_off_solution, impl_cnt_solution; 
  implP|impl_off_solution;
  implP|impl_cnt_solution;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  double *solution=(double *)(impl_buf+impl_off_solution);
  impl_obj->updateOFUB(ofub, size, solution);
  return implP.size();
}

void CkIndex_Master::_marshallmessagepup_updateOFUB_marshall3(PUP::er &implDestP,void *impl_msg) {
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double ofub, int size, const double *solution*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  int size; implP|size;
  int impl_off_solution, impl_cnt_solution; 
  implP|impl_off_solution;
  implP|impl_cnt_solution;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  double *solution=(double *)(impl_buf+impl_off_solution);
  if (implDestP.hasComments()) implDestP.comment("ofub");
  implDestP|ofub;
  if (implDestP.hasComments()) implDestP.comment("size");
  implDestP|size;
  if (implDestP.hasComments()) implDestP.comment("solution");
  implDestP.synchronize(PUP::sync_begin_array);
  for (int impl_i=0;impl_i*(sizeof(*solution))<impl_cnt_solution;impl_i++) {
    implDestP.synchronize(PUP::sync_item);
    implDestP|solution[impl_i];
  }
  implDestP.synchronize(PUP::sync_end_array);
}
PUPable_def(SINGLE_ARG(Closure_Master::updateOFUB_3_closure))
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void done(void);
 */

void CProxy_Master::done(void)
{
  ckCheck();
  void *impl_msg = CkAllocSysMsg();
  if (ckIsDelegated()) {
    int destPE=CkChareMsgPrep(CkIndex_Master::idx_done_void(), impl_msg, &ckGetChareID());
    if (destPE!=-1) ckDelegatedTo()->ChareSend(ckDelegatedPtr(),CkIndex_Master::idx_done_void(), impl_msg, &ckGetChareID(),destPE);
  }
  else CkSendMsg(CkIndex_Master::idx_done_void(), impl_msg, &ckGetChareID(),0);
}

// Entry point registration function

int CkIndex_Master::reg_done_void() {
  int epidx = CkRegisterEp("done(void)",
      _call_done_void, 0, __idx, 0);
  return epidx;
}


void CkIndex_Master::_call_done_void(void* impl_msg, void* impl_obj_void)
{
  Master* impl_obj = static_cast<Master *>(impl_obj_void);
  CkFreeSysMsg(impl_msg);
  impl_obj->done();
}
PUPable_def(SINGLE_ARG(Closure_Master::done_4_closure))
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void getStat(CkReductionMsg* impl_msg);
 */

void CProxy_Master::getStat(CkReductionMsg* impl_msg)
{
  ckCheck();
  if (ckIsDelegated()) {
    int destPE=CkChareMsgPrep(CkIndex_Master::idx_getStat_CkReductionMsg(), impl_msg, &ckGetChareID());
    if (destPE!=-1) ckDelegatedTo()->ChareSend(ckDelegatedPtr(),CkIndex_Master::idx_getStat_CkReductionMsg(), impl_msg, &ckGetChareID(),destPE);
  }
  else CkSendMsg(CkIndex_Master::idx_getStat_CkReductionMsg(), impl_msg, &ckGetChareID(),0);
}

void CkIndex_Master::_call_redn_wrapper_getStat_CkReductionMsg(void* impl_msg, void* impl_obj_void)
{
  Master* impl_obj = static_cast<Master*> (impl_obj_void);
  char* impl_buf = (char*)((CkReductionMsg*)impl_msg)->getData();
  impl_obj->getStat((CkReductionMsg*)impl_msg);
  delete (CkReductionMsg*)impl_msg;
}


// Entry point registration function

int CkIndex_Master::reg_getStat_CkReductionMsg() {
  int epidx = CkRegisterEp("getStat(CkReductionMsg* impl_msg)",
      _call_getStat_CkReductionMsg, CMessage_CkReductionMsg::__idx, __idx, 0);
  CkRegisterMessagePupFn(epidx, (CkMessagePupFn)CkReductionMsg::ckDebugPup);
  return epidx;
}


// Redn wrapper registration function

int CkIndex_Master::reg_redn_wrapper_getStat_CkReductionMsg() {
  return CkRegisterEp("redn_wrapper_getStat(CkReductionMsg *impl_msg)",
      _call_redn_wrapper_getStat_CkReductionMsg, CMessage_CkReductionMsg::__idx, __idx, 0);
}


void CkIndex_Master::_call_getStat_CkReductionMsg(void* impl_msg, void* impl_obj_void)
{
  Master* impl_obj = static_cast<Master *>(impl_obj_void);
  impl_obj->getStat((CkReductionMsg*)impl_msg);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
void CkIndex_Master::__register(const char *s, size_t size) {
  __idx = CkRegisterChare(s, size, TypeMainChare);
  CkRegisterBase(__idx, CkIndex_Chare::__idx);
  // REG: Master(CkArgMsg* impl_msg);
  idx_Master_CkArgMsg();
  CkRegisterMainChare(__idx, idx_Master_CkArgMsg());

  // REG: void groupCreated(CkReductionMsg* impl_msg);
  idx_groupCreated_CkReductionMsg();
  idx_redn_wrapper_groupCreated_CkReductionMsg();

  // REG: void updateOFUB(double ofub, int size, const double *solution);
  idx_updateOFUB_marshall3();

  // REG: void done(void);
  idx_done_void();

  // REG: void getStat(CkReductionMsg* impl_msg);
  idx_getStat_CkReductionMsg();
  idx_redn_wrapper_getStat_CkReductionMsg();

}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: nodegroup Cache: NodeGroup{
Cache(double ofub);
void updateOFUB(double ofub);
void postConstraint(const set<constraint_t > &constraints);
void postFinishedConstraint(const constraint_set_t &constraints);
void postDroppedConstraint(const set<constraint_set_t > &constraints);
void reportStat(void);
};
 */
#ifndef CK_TEMPLATES_ONLY
 int CkIndex_Cache::__idx=0;
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
/* DEFS: Cache(double ofub);
 */
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void updateOFUB(double ofub);
 */

void CProxyElement_Cache::updateOFUB(double ofub, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: double ofub
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|ofub;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|ofub;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSend(ckDelegatedPtr(),CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetGroupPe(), ckGetGroupID());
  } else CkSendMsgNodeBranch(CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetGroupPe(), ckGetGroupID(),0+CK_MSG_EXPEDITED);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postConstraint(const set<constraint_t > &constraints);
 */

void CProxyElement_Cache::postConstraint(const set<constraint_t > &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const set<constraint_t > &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_t > &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_t > &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSend(ckDelegatedPtr(),CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetGroupPe(), ckGetGroupID());
  } else CkSendMsgNodeBranch(CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetGroupPe(), ckGetGroupID(),0+CK_MSG_EXPEDITED);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postFinishedConstraint(const constraint_set_t &constraints);
 */

void CProxyElement_Cache::postFinishedConstraint(const constraint_set_t &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const constraint_set_t &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_set_t &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(constraint_set_t &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSend(ckDelegatedPtr(),CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetGroupPe(), ckGetGroupID());
  } else CkSendMsgNodeBranch(CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetGroupPe(), ckGetGroupID(),0+CK_MSG_EXPEDITED);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postDroppedConstraint(const set<constraint_set_t > &constraints);
 */

void CProxyElement_Cache::postDroppedConstraint(const set<constraint_set_t > &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const set<constraint_set_t > &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_set_t > &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_set_t > &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSend(ckDelegatedPtr(),CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetGroupPe(), ckGetGroupID());
  } else CkSendMsgNodeBranch(CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetGroupPe(), ckGetGroupID(),0+CK_MSG_EXPEDITED);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void reportStat(void);
 */

void CProxyElement_Cache::reportStat(void)
{
  ckCheck();
  void *impl_msg = CkAllocSysMsg();
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSend(ckDelegatedPtr(),CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetGroupPe(), ckGetGroupID());
  } else CkSendMsgNodeBranch(CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetGroupPe(), ckGetGroupID(),0);
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: Cache(double ofub);
 */

CkGroupID CProxy_Cache::ckNew(double ofub, const CkEntryOptions *impl_e_opts)
{
  //Marshall: double ofub
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|ofub;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|ofub;
  }
  UsrToEnv(impl_msg)->setMsgtype(NodeBocInitMsg);
  if (impl_e_opts)
    UsrToEnv(impl_msg)->setGroupDep(impl_e_opts->getGroupDepID());
  return CkCreateNodeGroup(CkIndex_Cache::__idx, CkIndex_Cache::idx_Cache_marshall1(), impl_msg);
}

  CProxy_Cache::CProxy_Cache(double ofub, const CkEntryOptions *impl_e_opts)
{
  //Marshall: double ofub
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|ofub;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|ofub;
  }
  UsrToEnv(impl_msg)->setMsgtype(NodeBocInitMsg);
  if (impl_e_opts)
    UsrToEnv(impl_msg)->setGroupDep(impl_e_opts->getGroupDepID());
  ckSetGroupID(CkCreateNodeGroup(CkIndex_Cache::__idx, CkIndex_Cache::idx_Cache_marshall1(), impl_msg));
}

// Entry point registration function

int CkIndex_Cache::reg_Cache_marshall1() {
  int epidx = CkRegisterEp("Cache(double ofub)",
      _call_Cache_marshall1, CkMarshallMsg::__idx, __idx, 0+CK_EP_NOKEEP);
  CkRegisterMarshallUnpackFn(epidx, _callmarshall_Cache_marshall1);
  CkRegisterMessagePupFn(epidx, _marshallmessagepup_Cache_marshall1);

  return epidx;
}


void CkIndex_Cache::_call_Cache_marshall1(void* impl_msg, void* impl_obj_void)
{
  Cache* impl_obj = static_cast<Cache *>(impl_obj_void);
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double ofub*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  new (impl_obj) Cache(ofub);
}

int CkIndex_Cache::_callmarshall_Cache_marshall1(char* impl_buf, void* impl_obj_void) {
  Cache* impl_obj = static_cast< Cache *>(impl_obj_void);
  /*Unmarshall pup'd fields: double ofub*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  new (impl_obj) Cache(ofub);
  return implP.size();
}

void CkIndex_Cache::_marshallmessagepup_Cache_marshall1(PUP::er &implDestP,void *impl_msg) {
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double ofub*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  if (implDestP.hasComments()) implDestP.comment("ofub");
  implDestP|ofub;
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void updateOFUB(double ofub);
 */

void CProxy_Cache::updateOFUB(double ofub, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: double ofub
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|ofub;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|ofub;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupBroadcast(ckDelegatedPtr(),CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetGroupID());
  } else CkBroadcastMsgNodeBranch(CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetGroupID(),0+CK_MSG_EXPEDITED);
}

// Entry point registration function

int CkIndex_Cache::reg_updateOFUB_marshall2() {
  int epidx = CkRegisterEp("updateOFUB(double ofub)",
      _call_updateOFUB_marshall2, CkMarshallMsg::__idx, __idx, 0+CK_EP_NOKEEP);
  CkRegisterMarshallUnpackFn(epidx, _callmarshall_updateOFUB_marshall2);
  CkRegisterMessagePupFn(epidx, _marshallmessagepup_updateOFUB_marshall2);

  return epidx;
}


void CkIndex_Cache::_call_updateOFUB_marshall2(void* impl_msg, void* impl_obj_void)
{
  Cache* impl_obj = static_cast<Cache *>(impl_obj_void);
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double ofub*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->updateOFUB(ofub);
}

int CkIndex_Cache::_callmarshall_updateOFUB_marshall2(char* impl_buf, void* impl_obj_void) {
  Cache* impl_obj = static_cast< Cache *>(impl_obj_void);
  /*Unmarshall pup'd fields: double ofub*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->updateOFUB(ofub);
  return implP.size();
}

void CkIndex_Cache::_marshallmessagepup_updateOFUB_marshall2(PUP::er &implDestP,void *impl_msg) {
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double ofub*/
  PUP::fromMem implP(impl_buf);
  double ofub; implP|ofub;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  if (implDestP.hasComments()) implDestP.comment("ofub");
  implDestP|ofub;
}
PUPable_def(SINGLE_ARG(Closure_Cache::updateOFUB_2_closure))
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postConstraint(const set<constraint_t > &constraints);
 */

void CProxy_Cache::postConstraint(const set<constraint_t > &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const set<constraint_t > &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_t > &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_t > &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupBroadcast(ckDelegatedPtr(),CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetGroupID());
  } else CkBroadcastMsgNodeBranch(CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetGroupID(),0+CK_MSG_EXPEDITED);
}

// Entry point registration function

int CkIndex_Cache::reg_postConstraint_marshall3() {
  int epidx = CkRegisterEp("postConstraint(const set<constraint_t > &constraints)",
      _call_postConstraint_marshall3, CkMarshallMsg::__idx, __idx, 0+CK_EP_NOKEEP);
  CkRegisterMarshallUnpackFn(epidx, _callmarshall_postConstraint_marshall3);
  CkRegisterMessagePupFn(epidx, _marshallmessagepup_postConstraint_marshall3);

  return epidx;
}


void CkIndex_Cache::_call_postConstraint_marshall3(void* impl_msg, void* impl_obj_void)
{
  Cache* impl_obj = static_cast<Cache *>(impl_obj_void);
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: const set<constraint_t > &constraints*/
  PUP::fromMem implP(impl_buf);
  set<constraint_t > constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->postConstraint(constraints);
}

int CkIndex_Cache::_callmarshall_postConstraint_marshall3(char* impl_buf, void* impl_obj_void) {
  Cache* impl_obj = static_cast< Cache *>(impl_obj_void);
  /*Unmarshall pup'd fields: const set<constraint_t > &constraints*/
  PUP::fromMem implP(impl_buf);
  set<constraint_t > constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->postConstraint(constraints);
  return implP.size();
}

void CkIndex_Cache::_marshallmessagepup_postConstraint_marshall3(PUP::er &implDestP,void *impl_msg) {
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: const set<constraint_t > &constraints*/
  PUP::fromMem implP(impl_buf);
  set<constraint_t > constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  if (implDestP.hasComments()) implDestP.comment("constraints");
  implDestP|constraints;
}
PUPable_def(SINGLE_ARG(Closure_Cache::postConstraint_3_closure))
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postFinishedConstraint(const constraint_set_t &constraints);
 */

void CProxy_Cache::postFinishedConstraint(const constraint_set_t &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const constraint_set_t &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_set_t &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(constraint_set_t &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupBroadcast(ckDelegatedPtr(),CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetGroupID());
  } else CkBroadcastMsgNodeBranch(CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetGroupID(),0+CK_MSG_EXPEDITED);
}

// Entry point registration function

int CkIndex_Cache::reg_postFinishedConstraint_marshall4() {
  int epidx = CkRegisterEp("postFinishedConstraint(const constraint_set_t &constraints)",
      _call_postFinishedConstraint_marshall4, CkMarshallMsg::__idx, __idx, 0+CK_EP_NOKEEP);
  CkRegisterMarshallUnpackFn(epidx, _callmarshall_postFinishedConstraint_marshall4);
  CkRegisterMessagePupFn(epidx, _marshallmessagepup_postFinishedConstraint_marshall4);

  return epidx;
}


void CkIndex_Cache::_call_postFinishedConstraint_marshall4(void* impl_msg, void* impl_obj_void)
{
  Cache* impl_obj = static_cast<Cache *>(impl_obj_void);
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: const constraint_set_t &constraints*/
  PUP::fromMem implP(impl_buf);
  constraint_set_t constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->postFinishedConstraint(constraints);
}

int CkIndex_Cache::_callmarshall_postFinishedConstraint_marshall4(char* impl_buf, void* impl_obj_void) {
  Cache* impl_obj = static_cast< Cache *>(impl_obj_void);
  /*Unmarshall pup'd fields: const constraint_set_t &constraints*/
  PUP::fromMem implP(impl_buf);
  constraint_set_t constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->postFinishedConstraint(constraints);
  return implP.size();
}

void CkIndex_Cache::_marshallmessagepup_postFinishedConstraint_marshall4(PUP::er &implDestP,void *impl_msg) {
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: const constraint_set_t &constraints*/
  PUP::fromMem implP(impl_buf);
  constraint_set_t constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  if (implDestP.hasComments()) implDestP.comment("constraints");
  implDestP|constraints;
}
PUPable_def(SINGLE_ARG(Closure_Cache::postFinishedConstraint_4_closure))
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postDroppedConstraint(const set<constraint_set_t > &constraints);
 */

void CProxy_Cache::postDroppedConstraint(const set<constraint_set_t > &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const set<constraint_set_t > &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_set_t > &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_set_t > &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupBroadcast(ckDelegatedPtr(),CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetGroupID());
  } else CkBroadcastMsgNodeBranch(CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetGroupID(),0+CK_MSG_EXPEDITED);
}

// Entry point registration function

int CkIndex_Cache::reg_postDroppedConstraint_marshall5() {
  int epidx = CkRegisterEp("postDroppedConstraint(const set<constraint_set_t > &constraints)",
      _call_postDroppedConstraint_marshall5, CkMarshallMsg::__idx, __idx, 0+CK_EP_NOKEEP);
  CkRegisterMarshallUnpackFn(epidx, _callmarshall_postDroppedConstraint_marshall5);
  CkRegisterMessagePupFn(epidx, _marshallmessagepup_postDroppedConstraint_marshall5);

  return epidx;
}


void CkIndex_Cache::_call_postDroppedConstraint_marshall5(void* impl_msg, void* impl_obj_void)
{
  Cache* impl_obj = static_cast<Cache *>(impl_obj_void);
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: const set<constraint_set_t > &constraints*/
  PUP::fromMem implP(impl_buf);
  set<constraint_set_t > constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->postDroppedConstraint(constraints);
}

int CkIndex_Cache::_callmarshall_postDroppedConstraint_marshall5(char* impl_buf, void* impl_obj_void) {
  Cache* impl_obj = static_cast< Cache *>(impl_obj_void);
  /*Unmarshall pup'd fields: const set<constraint_set_t > &constraints*/
  PUP::fromMem implP(impl_buf);
  set<constraint_set_t > constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  impl_obj->postDroppedConstraint(constraints);
  return implP.size();
}

void CkIndex_Cache::_marshallmessagepup_postDroppedConstraint_marshall5(PUP::er &implDestP,void *impl_msg) {
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: const set<constraint_set_t > &constraints*/
  PUP::fromMem implP(impl_buf);
  set<constraint_set_t > constraints; implP|constraints;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  if (implDestP.hasComments()) implDestP.comment("constraints");
  implDestP|constraints;
}
PUPable_def(SINGLE_ARG(Closure_Cache::postDroppedConstraint_5_closure))
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void reportStat(void);
 */

void CProxy_Cache::reportStat(void)
{
  ckCheck();
  void *impl_msg = CkAllocSysMsg();
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupBroadcast(ckDelegatedPtr(),CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetGroupID());
  } else CkBroadcastMsgNodeBranch(CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetGroupID(),0);
}

// Entry point registration function

int CkIndex_Cache::reg_reportStat_void() {
  int epidx = CkRegisterEp("reportStat(void)",
      _call_reportStat_void, 0, __idx, 0);
  return epidx;
}


void CkIndex_Cache::_call_reportStat_void(void* impl_msg, void* impl_obj_void)
{
  Cache* impl_obj = static_cast<Cache *>(impl_obj_void);
  CkFreeSysMsg(impl_msg);
  impl_obj->reportStat();
}
PUPable_def(SINGLE_ARG(Closure_Cache::reportStat_6_closure))
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: Cache(double ofub);
 */
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void updateOFUB(double ofub);
 */

void CProxySection_Cache::updateOFUB(double ofub, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: double ofub
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|ofub;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|ofub;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSectionSend(ckDelegatedPtr(),CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg, ckGetNumSections(), ckGetSectionIDs());
  } else {
    void *impl_msg_tmp = (ckGetNumSections()>1) ? CkCopyMsg((void **) &impl_msg) : impl_msg;
    for (int i=0; i<ckGetNumSections(); ++i) {
       impl_msg_tmp= (i<ckGetNumSections()-1) ? CkCopyMsg((void **) &impl_msg):impl_msg;
       CkSendMsgNodeBranchMulti(CkIndex_Cache::idx_updateOFUB_marshall2(), impl_msg_tmp, ckGetGroupIDn(i), ckGetNumElements(i), ckGetElements(i),0+CK_MSG_EXPEDITED);
    }
  }
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postConstraint(const set<constraint_t > &constraints);
 */

void CProxySection_Cache::postConstraint(const set<constraint_t > &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const set<constraint_t > &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_t > &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_t > &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSectionSend(ckDelegatedPtr(),CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg, ckGetNumSections(), ckGetSectionIDs());
  } else {
    void *impl_msg_tmp = (ckGetNumSections()>1) ? CkCopyMsg((void **) &impl_msg) : impl_msg;
    for (int i=0; i<ckGetNumSections(); ++i) {
       impl_msg_tmp= (i<ckGetNumSections()-1) ? CkCopyMsg((void **) &impl_msg):impl_msg;
       CkSendMsgNodeBranchMulti(CkIndex_Cache::idx_postConstraint_marshall3(), impl_msg_tmp, ckGetGroupIDn(i), ckGetNumElements(i), ckGetElements(i),0+CK_MSG_EXPEDITED);
    }
  }
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postFinishedConstraint(const constraint_set_t &constraints);
 */

void CProxySection_Cache::postFinishedConstraint(const constraint_set_t &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const constraint_set_t &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_set_t &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(constraint_set_t &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSectionSend(ckDelegatedPtr(),CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg, ckGetNumSections(), ckGetSectionIDs());
  } else {
    void *impl_msg_tmp = (ckGetNumSections()>1) ? CkCopyMsg((void **) &impl_msg) : impl_msg;
    for (int i=0; i<ckGetNumSections(); ++i) {
       impl_msg_tmp= (i<ckGetNumSections()-1) ? CkCopyMsg((void **) &impl_msg):impl_msg;
       CkSendMsgNodeBranchMulti(CkIndex_Cache::idx_postFinishedConstraint_marshall4(), impl_msg_tmp, ckGetGroupIDn(i), ckGetNumElements(i), ckGetElements(i),0+CK_MSG_EXPEDITED);
    }
  }
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void postDroppedConstraint(const set<constraint_set_t > &constraints);
 */

void CProxySection_Cache::postDroppedConstraint(const set<constraint_set_t > &constraints, const CkEntryOptions *impl_e_opts)
{
  ckCheck();
  //Marshall: const set<constraint_set_t > &constraints
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_set_t > &)constraints;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    //Have to cast away const-ness to get pup routine
    implP|(set<constraint_set_t > &)constraints;
  }
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSectionSend(ckDelegatedPtr(),CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg, ckGetNumSections(), ckGetSectionIDs());
  } else {
    void *impl_msg_tmp = (ckGetNumSections()>1) ? CkCopyMsg((void **) &impl_msg) : impl_msg;
    for (int i=0; i<ckGetNumSections(); ++i) {
       impl_msg_tmp= (i<ckGetNumSections()-1) ? CkCopyMsg((void **) &impl_msg):impl_msg;
       CkSendMsgNodeBranchMulti(CkIndex_Cache::idx_postDroppedConstraint_marshall5(), impl_msg_tmp, ckGetGroupIDn(i), ckGetNumElements(i), ckGetElements(i),0+CK_MSG_EXPEDITED);
    }
  }
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
/* DEFS: void reportStat(void);
 */

void CProxySection_Cache::reportStat(void)
{
  ckCheck();
  void *impl_msg = CkAllocSysMsg();
  if (ckIsDelegated()) {
     CkNodeGroupMsgPrep(CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetGroupID());
     ckDelegatedTo()->NodeGroupSectionSend(ckDelegatedPtr(),CkIndex_Cache::idx_reportStat_void(), impl_msg, ckGetNumSections(), ckGetSectionIDs());
  } else {
    void *impl_msg_tmp = (ckGetNumSections()>1) ? CkCopyMsg((void **) &impl_msg) : impl_msg;
    for (int i=0; i<ckGetNumSections(); ++i) {
       impl_msg_tmp= (i<ckGetNumSections()-1) ? CkCopyMsg((void **) &impl_msg):impl_msg;
       CkSendMsgNodeBranchMulti(CkIndex_Cache::idx_reportStat_void(), impl_msg_tmp, ckGetGroupIDn(i), ckGetNumElements(i), ckGetElements(i),0);
    }
  }
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
void CkIndex_Cache::__register(const char *s, size_t size) {
  __idx = CkRegisterChare(s, size, TypeGroup);
  CkRegisterBase(__idx, CkIndex_NodeGroup::__idx);
   CkRegisterGroupIrr(__idx,Cache::isIrreducible());
  // REG: Cache(double ofub);
  idx_Cache_marshall1();

  // REG: void updateOFUB(double ofub);
  idx_updateOFUB_marshall2();

  // REG: void postConstraint(const set<constraint_t > &constraints);
  idx_postConstraint_marshall3();

  // REG: void postFinishedConstraint(const constraint_set_t &constraints);
  idx_postFinishedConstraint_marshall4();

  // REG: void postDroppedConstraint(const set<constraint_set_t > &constraints);
  idx_postDroppedConstraint_marshall5();

  // REG: void reportStat(void);
  idx_reportStat_void();

}
#endif /* CK_TEMPLATES_ONLY */

/* DEFS: chare Slave: Chare{
Slave(double parent_cost, const constraint_t &constraints, int branch_col);
};
 */
#ifndef CK_TEMPLATES_ONLY
 int CkIndex_Slave::__idx=0;
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
/* DEFS: Slave(double parent_cost, const constraint_t &constraints, int branch_col);
 */

CkChareID CProxy_Slave::ckNew(double parent_cost, const constraint_t &constraints, int branch_col, int impl_onPE, const CkEntryOptions *impl_e_opts)
{
  //Marshall: double parent_cost, const constraint_t &constraints, int branch_col
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|parent_cost;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_t &)constraints;
    implP|branch_col;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|parent_cost;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_t &)constraints;
    implP|branch_col;
  }
  CkChareID impl_ret;
  CkCreateChare(CkIndex_Slave::__idx, CkIndex_Slave::idx_Slave_marshall1(), impl_msg, &impl_ret, impl_onPE);
  return impl_ret;
}

void CProxy_Slave::ckNew(double parent_cost, const constraint_t &constraints, int branch_col, CkChareID* pcid, int impl_onPE, const CkEntryOptions *impl_e_opts)
{
  //Marshall: double parent_cost, const constraint_t &constraints, int branch_col
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|parent_cost;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_t &)constraints;
    implP|branch_col;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|parent_cost;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_t &)constraints;
    implP|branch_col;
  }
  CkCreateChare(CkIndex_Slave::__idx, CkIndex_Slave::idx_Slave_marshall1(), impl_msg, pcid, impl_onPE);
}

  CProxy_Slave::CProxy_Slave(double parent_cost, const constraint_t &constraints, int branch_col, int impl_onPE, const CkEntryOptions *impl_e_opts)
{
  //Marshall: double parent_cost, const constraint_t &constraints, int branch_col
  int impl_off=0;
  { //Find the size of the PUP'd data
    PUP::sizer implP;
    implP|parent_cost;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_t &)constraints;
    implP|branch_col;
    impl_off+=implP.size();
  }
  CkMarshallMsg *impl_msg=CkAllocateMarshallMsg(impl_off,impl_e_opts);
  { //Copy over the PUP'd data
    PUP::toMem implP((void *)impl_msg->msgBuf);
    implP|parent_cost;
    //Have to cast away const-ness to get pup routine
    implP|(constraint_t &)constraints;
    implP|branch_col;
  }
  CkChareID impl_ret;
  CkCreateChare(CkIndex_Slave::__idx, CkIndex_Slave::idx_Slave_marshall1(), impl_msg, &impl_ret, impl_onPE);
  ckSetChareID(impl_ret);
}

// Entry point registration function

int CkIndex_Slave::reg_Slave_marshall1() {
  int epidx = CkRegisterEp("Slave(double parent_cost, const constraint_t &constraints, int branch_col)",
      _call_Slave_marshall1, CkMarshallMsg::__idx, __idx, 0+CK_EP_NOKEEP);
  CkRegisterMarshallUnpackFn(epidx, _callmarshall_Slave_marshall1);
  CkRegisterMessagePupFn(epidx, _marshallmessagepup_Slave_marshall1);

  return epidx;
}


void CkIndex_Slave::_call_Slave_marshall1(void* impl_msg, void* impl_obj_void)
{
  Slave* impl_obj = static_cast<Slave *>(impl_obj_void);
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double parent_cost, const constraint_t &constraints, int branch_col*/
  PUP::fromMem implP(impl_buf);
  double parent_cost; implP|parent_cost;
  constraint_t constraints; implP|constraints;
  int branch_col; implP|branch_col;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  new (impl_obj) Slave(parent_cost, constraints, branch_col);
}

int CkIndex_Slave::_callmarshall_Slave_marshall1(char* impl_buf, void* impl_obj_void) {
  Slave* impl_obj = static_cast< Slave *>(impl_obj_void);
  /*Unmarshall pup'd fields: double parent_cost, const constraint_t &constraints, int branch_col*/
  PUP::fromMem implP(impl_buf);
  double parent_cost; implP|parent_cost;
  constraint_t constraints; implP|constraints;
  int branch_col; implP|branch_col;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  new (impl_obj) Slave(parent_cost, constraints, branch_col);
  return implP.size();
}

void CkIndex_Slave::_marshallmessagepup_Slave_marshall1(PUP::er &implDestP,void *impl_msg) {
  CkMarshallMsg *impl_msg_typed=(CkMarshallMsg *)impl_msg;
  char *impl_buf=impl_msg_typed->msgBuf;
  /*Unmarshall pup'd fields: double parent_cost, const constraint_t &constraints, int branch_col*/
  PUP::fromMem implP(impl_buf);
  double parent_cost; implP|parent_cost;
  constraint_t constraints; implP|constraints;
  int branch_col; implP|branch_col;
  impl_buf+=CK_ALIGN(implP.size(),16);
  /*Unmarshall arrays:*/
  if (implDestP.hasComments()) implDestP.comment("parent_cost");
  implDestP|parent_cost;
  if (implDestP.hasComments()) implDestP.comment("constraints");
  implDestP|constraints;
  if (implDestP.hasComments()) implDestP.comment("branch_col");
  implDestP|branch_col;
}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
void CkIndex_Slave::__register(const char *s, size_t size) {
  __idx = CkRegisterChare(s, size, TypeChare);
  CkRegisterBase(__idx, CkIndex_Chare::__idx);
  // REG: Slave(double parent_cost, const constraint_t &constraints, int branch_col);
  idx_Slave_marshall1();

}
#endif /* CK_TEMPLATES_ONLY */

#ifndef CK_TEMPLATES_ONLY
void _registerilpprune(void)
{
  static int _done = 0; if(_done) return; _done = 1;
  CkRegisterReadonly("mainProxy","CProxy_Master",sizeof(mainProxy),(void *) &mainProxy,__xlater_roPup_mainProxy);

  CkRegisterReadonly("cacheProxy","CProxy_Cache",sizeof(cacheProxy),(void *) &cacheProxy,__xlater_roPup_cacheProxy);

  CkRegisterReadonly("problem","map_t",sizeof(problem),(void *) &problem,__xlater_roPup_problem);

  CkRegisterReadonly("dist","distance_t",sizeof(dist),(void *) &dist,__xlater_roPup_dist);

  CkRegisterReadonly("variable_map","idx_map_t",sizeof(variable_map),(void *) &variable_map,__xlater_roPup_variable_map);

  CkRegisterReadonly("objective_vec","double_vec_t",sizeof(objective_vec),(void *) &objective_vec,__xlater_roPup_objective_vec);

  CkRegisterReadonly("col_lb_vec","double_vec_t",sizeof(col_lb_vec),(void *) &col_lb_vec,__xlater_roPup_col_lb_vec);

  CkRegisterReadonly("col_ub_vec","double_vec_t",sizeof(col_ub_vec),(void *) &col_ub_vec,__xlater_roPup_col_ub_vec);

  CkRegisterReadonly("base_constraint_vec","constraint_vec_t",sizeof(base_constraint_vec),(void *) &base_constraint_vec,__xlater_roPup_base_constraint_vec);

  CkRegisterReadonly("n_cols","int",sizeof(n_cols),(void *) &n_cols,__xlater_roPup_n_cols);

  CkRegisterReadonly("cache_size","int",sizeof(cache_size),(void *) &cache_size,__xlater_roPup_cache_size);

  CkRegisterReadonly("granularity","int",sizeof(granularity),(void *) &granularity,__xlater_roPup_granularity);

/* REG: mainchare Master: Chare{
Master(CkArgMsg* impl_msg);
void groupCreated(CkReductionMsg* impl_msg);
void updateOFUB(double ofub, int size, const double *solution);
void done(void);
void getStat(CkReductionMsg* impl_msg);
};
*/
  CkIndex_Master::__register("Master", sizeof(Master));

/* REG: nodegroup Cache: NodeGroup{
Cache(double ofub);
void updateOFUB(double ofub);
void postConstraint(const set<constraint_t > &constraints);
void postFinishedConstraint(const constraint_set_t &constraints);
void postDroppedConstraint(const set<constraint_set_t > &constraints);
void reportStat(void);
};
*/
  CkIndex_Cache::__register("Cache", sizeof(Cache));

/* REG: chare Slave: Chare{
Slave(double parent_cost, const constraint_t &constraints, int branch_col);
};
*/
  CkIndex_Slave::__register("Slave", sizeof(Slave));

}
extern "C" void CkRegisterMainModule(void) {
  _registerilpprune();
}
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
template <>
void CBase_Master::virtual_pup(PUP::er &p) {
    recursive_pup<Master >(dynamic_cast<Master* >(this), p);
}
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
template <>
void CBase_Cache::virtual_pup(PUP::er &p) {
    recursive_pup<Cache >(dynamic_cast<Cache* >(this), p);
}
#endif /* CK_TEMPLATES_ONLY */
#ifndef CK_TEMPLATES_ONLY
template <>
void CBase_Slave::virtual_pup(PUP::er &p) {
    recursive_pup<Slave >(dynamic_cast<Slave* >(this), p);
}
#endif /* CK_TEMPLATES_ONLY */
