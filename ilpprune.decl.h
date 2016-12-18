#ifndef _DECL_ilpprune_H_
#define _DECL_ilpprune_H_
#include "charm++.h"
#include "envelope.h"
#include <memory>
#include "sdag.h"
/* DECLS: readonly CProxy_Master mainProxy;
 */

/* DECLS: readonly CProxy_Cache cacheProxy;
 */

/* DECLS: readonly map_t problem;
 */

/* DECLS: readonly distance_t dist;
 */

/* DECLS: readonly idx_map_t variable_map;
 */

/* DECLS: readonly double_vec_t objective_vec;
 */

/* DECLS: readonly double_vec_t col_lb_vec;
 */

/* DECLS: readonly double_vec_t col_ub_vec;
 */

/* DECLS: readonly constraint_vec_t base_constraint_vec;
 */

/* DECLS: readonly int n_cols;
 */

/* DECLS: readonly int cache_size;
 */

/* DECLS: readonly int granularity;
 */

/* DECLS: mainchare Master: Chare{
Master(CkArgMsg* impl_msg);
void groupCreated(CkReductionMsg* impl_msg);
void updateOFUB(double ofub, int size, const double *solution);
void done(void);
void getStat(CkReductionMsg* impl_msg);
};
 */
 class Master;
 class CkIndex_Master;
 class CProxy_Master;
/* --------------- index object ------------------ */
class CkIndex_Master:public CkIndex_Chare{
  public:
    typedef Master local_t;
    typedef CkIndex_Master index_t;
    typedef CProxy_Master proxy_t;
    typedef CProxy_Master element_t;

    static int __idx;
    static void __register(const char *s, size_t size);
    /* DECLS: Master(CkArgMsg* impl_msg);
     */
    // Entry point registration at startup
    
    static int reg_Master_CkArgMsg();
    // Entry point index lookup
    
    inline static int idx_Master_CkArgMsg() {
      static int epidx = reg_Master_CkArgMsg();
      return epidx;
    }

    
    static int ckNew(CkArgMsg* impl_msg) { return idx_Master_CkArgMsg(); }
    
    static void _call_Master_CkArgMsg(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_Master_CkArgMsg(void* impl_msg, void* impl_obj);
    /* DECLS: void groupCreated(CkReductionMsg* impl_msg);
     */
    // Entry point registration at startup
    
    static int reg_groupCreated_CkReductionMsg();
    // Entry point index lookup
    
    inline static int idx_groupCreated_CkReductionMsg() {
      static int epidx = reg_groupCreated_CkReductionMsg();
      return epidx;
    }

    
    inline static int idx_groupCreated(void (Master::*)(CkReductionMsg* impl_msg) ) {
      return idx_groupCreated_CkReductionMsg();
    }


    
    static int groupCreated(CkReductionMsg* impl_msg) { return idx_groupCreated_CkReductionMsg(); }
    // Entry point registration at startup
    
    static int reg_redn_wrapper_groupCreated_CkReductionMsg();
    // Entry point index lookup
    
    inline static int idx_redn_wrapper_groupCreated_CkReductionMsg() {
      static int epidx = reg_redn_wrapper_groupCreated_CkReductionMsg();
      return epidx;
    }
    
    static int redn_wrapper_groupCreated(CkReductionMsg* impl_msg) { return idx_redn_wrapper_groupCreated_CkReductionMsg(); }
    
    static void _call_redn_wrapper_groupCreated_CkReductionMsg(void* impl_msg, void* impl_obj_void);
    
    static void _call_groupCreated_CkReductionMsg(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_groupCreated_CkReductionMsg(void* impl_msg, void* impl_obj);
    /* DECLS: void updateOFUB(double ofub, int size, const double *solution);
     */
    // Entry point registration at startup
    
    static int reg_updateOFUB_marshall3();
    // Entry point index lookup
    
    inline static int idx_updateOFUB_marshall3() {
      static int epidx = reg_updateOFUB_marshall3();
      return epidx;
    }

    
    inline static int idx_updateOFUB(void (Master::*)(double ofub, int size, const double *solution) ) {
      return idx_updateOFUB_marshall3();
    }


    
    static int updateOFUB(double ofub, int size, const double *solution) { return idx_updateOFUB_marshall3(); }
    
    static void _call_updateOFUB_marshall3(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_updateOFUB_marshall3(void* impl_msg, void* impl_obj);
    
    static int _callmarshall_updateOFUB_marshall3(char* impl_buf, void* impl_obj_void);
    
    static void _marshallmessagepup_updateOFUB_marshall3(PUP::er &p,void *msg);
    /* DECLS: void done(void);
     */
    // Entry point registration at startup
    
    static int reg_done_void();
    // Entry point index lookup
    
    inline static int idx_done_void() {
      static int epidx = reg_done_void();
      return epidx;
    }

    
    inline static int idx_done(void (Master::*)(void) ) {
      return idx_done_void();
    }


    
    static int done(void) { return idx_done_void(); }
    
    static void _call_done_void(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_done_void(void* impl_msg, void* impl_obj);
    /* DECLS: void getStat(CkReductionMsg* impl_msg);
     */
    // Entry point registration at startup
    
    static int reg_getStat_CkReductionMsg();
    // Entry point index lookup
    
    inline static int idx_getStat_CkReductionMsg() {
      static int epidx = reg_getStat_CkReductionMsg();
      return epidx;
    }

    
    inline static int idx_getStat(void (Master::*)(CkReductionMsg* impl_msg) ) {
      return idx_getStat_CkReductionMsg();
    }


    
    static int getStat(CkReductionMsg* impl_msg) { return idx_getStat_CkReductionMsg(); }
    // Entry point registration at startup
    
    static int reg_redn_wrapper_getStat_CkReductionMsg();
    // Entry point index lookup
    
    inline static int idx_redn_wrapper_getStat_CkReductionMsg() {
      static int epidx = reg_redn_wrapper_getStat_CkReductionMsg();
      return epidx;
    }
    
    static int redn_wrapper_getStat(CkReductionMsg* impl_msg) { return idx_redn_wrapper_getStat_CkReductionMsg(); }
    
    static void _call_redn_wrapper_getStat_CkReductionMsg(void* impl_msg, void* impl_obj_void);
    
    static void _call_getStat_CkReductionMsg(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_getStat_CkReductionMsg(void* impl_msg, void* impl_obj);
};
/* --------------- element proxy ------------------ */
class CProxy_Master:public CProxy_Chare{
  public:
    typedef Master local_t;
    typedef CkIndex_Master index_t;
    typedef CProxy_Master proxy_t;
    typedef CProxy_Master element_t;

    CProxy_Master(void) {};
    CProxy_Master(CkChareID __cid) : CProxy_Chare(__cid){  }
    CProxy_Master(const Chare *c) : CProxy_Chare(c){  }

    int ckIsDelegated(void) const
    { return CProxy_Chare::ckIsDelegated(); }
    inline CkDelegateMgr *ckDelegatedTo(void) const
    { return CProxy_Chare::ckDelegatedTo(); }
    inline CkDelegateData *ckDelegatedPtr(void) const
    { return CProxy_Chare::ckDelegatedPtr(); }
    CkGroupID ckDelegatedIdx(void) const
    { return CProxy_Chare::ckDelegatedIdx(); }

    inline void ckCheck(void) const
    { CProxy_Chare::ckCheck(); }
    const CkChareID &ckGetChareID(void) const
    { return CProxy_Chare::ckGetChareID(); }
    operator const CkChareID &(void) const
    { return ckGetChareID(); }

    void ckDelegate(CkDelegateMgr *dTo,CkDelegateData *dPtr=NULL)
    {       CProxy_Chare::ckDelegate(dTo,dPtr); }
    void ckUndelegate(void)
    {       CProxy_Chare::ckUndelegate(); }
    void pup(PUP::er &p)
    {       CProxy_Chare::pup(p); }

    void ckSetChareID(const CkChareID &c)
    {      CProxy_Chare::ckSetChareID(c); }
    Master *ckLocal(void) const
    { return (Master *)CkLocalChare(&ckGetChareID()); }
/* DECLS: Master(CkArgMsg* impl_msg);
 */
    static CkChareID ckNew(CkArgMsg* impl_msg, int onPE=CK_PE_ANY);
    static void ckNew(CkArgMsg* impl_msg, CkChareID* pcid, int onPE=CK_PE_ANY);
    CProxy_Master(CkArgMsg* impl_msg, int onPE=CK_PE_ANY);

/* DECLS: void groupCreated(CkReductionMsg* impl_msg);
 */
    
    void groupCreated(CkReductionMsg* impl_msg);

/* DECLS: void updateOFUB(double ofub, int size, const double *solution);
 */
    
    void updateOFUB(double ofub, int size, const double *solution, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void done(void);
 */
    
    void done(void);

/* DECLS: void getStat(CkReductionMsg* impl_msg);
 */
    
    void getStat(CkReductionMsg* impl_msg);

};
PUPmarshall(CProxy_Master)
#define Master_SDAG_CODE 
typedef CBaseT1<Chare, CProxy_Master>CBase_Master;

/* DECLS: nodegroup Cache: NodeGroup{
Cache(double ofub);
void updateOFUB(double ofub);
void postConstraint(const set<constraint_t > &constraints);
void postFinishedConstraint(const constraint_set_t &constraints);
void postDroppedConstraint(const set<constraint_set_t > &constraints);
void reportStat(void);
};
 */
 class Cache;
 class CkIndex_Cache;
 class CProxy_Cache;
 class CProxyElement_Cache;
 class CProxySection_Cache;
/* --------------- index object ------------------ */
class CkIndex_Cache:public CkIndex_NodeGroup{
  public:
    typedef Cache local_t;
    typedef CkIndex_Cache index_t;
    typedef CProxy_Cache proxy_t;
    typedef CProxyElement_Cache element_t;
    typedef CProxySection_Cache section_t;

    static int __idx;
    static void __register(const char *s, size_t size);
    /* DECLS: Cache(double ofub);
     */
    // Entry point registration at startup
    
    static int reg_Cache_marshall1();
    // Entry point index lookup
    
    inline static int idx_Cache_marshall1() {
      static int epidx = reg_Cache_marshall1();
      return epidx;
    }

    
    static int ckNew(double ofub) { return idx_Cache_marshall1(); }
    
    static void _call_Cache_marshall1(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_Cache_marshall1(void* impl_msg, void* impl_obj);
    
    static int _callmarshall_Cache_marshall1(char* impl_buf, void* impl_obj_void);
    
    static void _marshallmessagepup_Cache_marshall1(PUP::er &p,void *msg);
    /* DECLS: void updateOFUB(double ofub);
     */
    // Entry point registration at startup
    
    static int reg_updateOFUB_marshall2();
    // Entry point index lookup
    
    inline static int idx_updateOFUB_marshall2() {
      static int epidx = reg_updateOFUB_marshall2();
      return epidx;
    }

    
    inline static int idx_updateOFUB(void (Cache::*)(double ofub) ) {
      return idx_updateOFUB_marshall2();
    }


    
    static int updateOFUB(double ofub) { return idx_updateOFUB_marshall2(); }
    
    static void _call_updateOFUB_marshall2(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_updateOFUB_marshall2(void* impl_msg, void* impl_obj);
    
    static int _callmarshall_updateOFUB_marshall2(char* impl_buf, void* impl_obj_void);
    
    static void _marshallmessagepup_updateOFUB_marshall2(PUP::er &p,void *msg);
    /* DECLS: void postConstraint(const set<constraint_t > &constraints);
     */
    // Entry point registration at startup
    
    static int reg_postConstraint_marshall3();
    // Entry point index lookup
    
    inline static int idx_postConstraint_marshall3() {
      static int epidx = reg_postConstraint_marshall3();
      return epidx;
    }

    
    inline static int idx_postConstraint(void (Cache::*)(const set<constraint_t > &constraints) ) {
      return idx_postConstraint_marshall3();
    }


    
    static int postConstraint(const set<constraint_t > &constraints) { return idx_postConstraint_marshall3(); }
    
    static void _call_postConstraint_marshall3(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_postConstraint_marshall3(void* impl_msg, void* impl_obj);
    
    static int _callmarshall_postConstraint_marshall3(char* impl_buf, void* impl_obj_void);
    
    static void _marshallmessagepup_postConstraint_marshall3(PUP::er &p,void *msg);
    /* DECLS: void postFinishedConstraint(const constraint_set_t &constraints);
     */
    // Entry point registration at startup
    
    static int reg_postFinishedConstraint_marshall4();
    // Entry point index lookup
    
    inline static int idx_postFinishedConstraint_marshall4() {
      static int epidx = reg_postFinishedConstraint_marshall4();
      return epidx;
    }

    
    inline static int idx_postFinishedConstraint(void (Cache::*)(const constraint_set_t &constraints) ) {
      return idx_postFinishedConstraint_marshall4();
    }


    
    static int postFinishedConstraint(const constraint_set_t &constraints) { return idx_postFinishedConstraint_marshall4(); }
    
    static void _call_postFinishedConstraint_marshall4(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_postFinishedConstraint_marshall4(void* impl_msg, void* impl_obj);
    
    static int _callmarshall_postFinishedConstraint_marshall4(char* impl_buf, void* impl_obj_void);
    
    static void _marshallmessagepup_postFinishedConstraint_marshall4(PUP::er &p,void *msg);
    /* DECLS: void postDroppedConstraint(const set<constraint_set_t > &constraints);
     */
    // Entry point registration at startup
    
    static int reg_postDroppedConstraint_marshall5();
    // Entry point index lookup
    
    inline static int idx_postDroppedConstraint_marshall5() {
      static int epidx = reg_postDroppedConstraint_marshall5();
      return epidx;
    }

    
    inline static int idx_postDroppedConstraint(void (Cache::*)(const set<constraint_set_t > &constraints) ) {
      return idx_postDroppedConstraint_marshall5();
    }


    
    static int postDroppedConstraint(const set<constraint_set_t > &constraints) { return idx_postDroppedConstraint_marshall5(); }
    
    static void _call_postDroppedConstraint_marshall5(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_postDroppedConstraint_marshall5(void* impl_msg, void* impl_obj);
    
    static int _callmarshall_postDroppedConstraint_marshall5(char* impl_buf, void* impl_obj_void);
    
    static void _marshallmessagepup_postDroppedConstraint_marshall5(PUP::er &p,void *msg);
    /* DECLS: void reportStat(void);
     */
    // Entry point registration at startup
    
    static int reg_reportStat_void();
    // Entry point index lookup
    
    inline static int idx_reportStat_void() {
      static int epidx = reg_reportStat_void();
      return epidx;
    }

    
    inline static int idx_reportStat(void (Cache::*)(void) ) {
      return idx_reportStat_void();
    }


    
    static int reportStat(void) { return idx_reportStat_void(); }
    
    static void _call_reportStat_void(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_reportStat_void(void* impl_msg, void* impl_obj);
};
/* --------------- element proxy ------------------ */
class CProxyElement_Cache: public CProxyElement_NodeGroup{
  public:
    typedef Cache local_t;
    typedef CkIndex_Cache index_t;
    typedef CProxy_Cache proxy_t;
    typedef CProxyElement_Cache element_t;
    typedef CProxySection_Cache section_t;

    CProxyElement_Cache(void) {}
    CProxyElement_Cache(const IrrGroup *g) : CProxyElement_NodeGroup(g){  }
    CProxyElement_Cache(CkGroupID _gid,int _onPE,CK_DELCTOR_PARAM) : CProxyElement_NodeGroup(_gid,_onPE,CK_DELCTOR_ARGS){  }
    CProxyElement_Cache(CkGroupID _gid,int _onPE) : CProxyElement_NodeGroup(_gid,_onPE){  }

    int ckIsDelegated(void) const
    { return CProxyElement_NodeGroup::ckIsDelegated(); }
    inline CkDelegateMgr *ckDelegatedTo(void) const
    { return CProxyElement_NodeGroup::ckDelegatedTo(); }
    inline CkDelegateData *ckDelegatedPtr(void) const
    { return CProxyElement_NodeGroup::ckDelegatedPtr(); }
    CkGroupID ckDelegatedIdx(void) const
    { return CProxyElement_NodeGroup::ckDelegatedIdx(); }
inline void ckCheck(void) const {CProxyElement_NodeGroup::ckCheck();}
CkChareID ckGetChareID(void) const
   {return CProxyElement_NodeGroup::ckGetChareID();}
CkGroupID ckGetGroupID(void) const
   {return CProxyElement_NodeGroup::ckGetGroupID();}
operator CkGroupID () const { return ckGetGroupID(); }

    inline void setReductionClient(CkReductionClientFn fn,void *param=NULL) const
    { CProxyElement_NodeGroup::setReductionClient(fn,param); }
    inline void ckSetReductionClient(CkReductionClientFn fn,void *param=NULL) const
    { CProxyElement_NodeGroup::ckSetReductionClient(fn,param); }
    inline void ckSetReductionClient(CkCallback *cb) const
    { CProxyElement_NodeGroup::ckSetReductionClient(cb); }
int ckGetGroupPe(void) const
{return CProxyElement_NodeGroup::ckGetGroupPe();}

    void ckDelegate(CkDelegateMgr *dTo,CkDelegateData *dPtr=NULL)
    {       CProxyElement_NodeGroup::ckDelegate(dTo,dPtr); }
    void ckUndelegate(void)
    {       CProxyElement_NodeGroup::ckUndelegate(); }
    void pup(PUP::er &p)
    {       CProxyElement_NodeGroup::pup(p); }
    void ckSetGroupID(CkGroupID g) {
      CProxyElement_NodeGroup::ckSetGroupID(g);
    }
    Cache* ckLocalBranch(void) const {
      return ckLocalBranch(ckGetGroupID());
    }
    static Cache* ckLocalBranch(CkGroupID gID) {
      return (Cache*)CkLocalNodeBranch(gID);
    }
/* DECLS: Cache(double ofub);
 */
    

/* DECLS: void updateOFUB(double ofub);
 */
    
    void updateOFUB(double ofub, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postConstraint(const set<constraint_t > &constraints);
 */
    
    void postConstraint(const set<constraint_t > &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postFinishedConstraint(const constraint_set_t &constraints);
 */
    
    void postFinishedConstraint(const constraint_set_t &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postDroppedConstraint(const set<constraint_set_t > &constraints);
 */
    
    void postDroppedConstraint(const set<constraint_set_t > &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void reportStat(void);
 */
    
    void reportStat(void);

};
PUPmarshall(CProxyElement_Cache)
/* ---------------- collective proxy -------------- */
class CProxy_Cache: public CProxy_NodeGroup{
  public:
    typedef Cache local_t;
    typedef CkIndex_Cache index_t;
    typedef CProxy_Cache proxy_t;
    typedef CProxyElement_Cache element_t;
    typedef CProxySection_Cache section_t;

    CProxy_Cache(void) {}
    CProxy_Cache(const IrrGroup *g) : CProxy_NodeGroup(g){  }
    CProxy_Cache(CkGroupID _gid,CK_DELCTOR_PARAM) : CProxy_NodeGroup(_gid,CK_DELCTOR_ARGS){  }
    CProxy_Cache(CkGroupID _gid) : CProxy_NodeGroup(_gid){  }
    CProxyElement_Cache operator[](int onPE) const
      {return CProxyElement_Cache(ckGetGroupID(),onPE,CK_DELCTOR_CALL);}

    int ckIsDelegated(void) const
    { return CProxy_NodeGroup::ckIsDelegated(); }
    inline CkDelegateMgr *ckDelegatedTo(void) const
    { return CProxy_NodeGroup::ckDelegatedTo(); }
    inline CkDelegateData *ckDelegatedPtr(void) const
    { return CProxy_NodeGroup::ckDelegatedPtr(); }
    CkGroupID ckDelegatedIdx(void) const
    { return CProxy_NodeGroup::ckDelegatedIdx(); }
inline void ckCheck(void) const {CProxy_NodeGroup::ckCheck();}
CkChareID ckGetChareID(void) const
   {return CProxy_NodeGroup::ckGetChareID();}
CkGroupID ckGetGroupID(void) const
   {return CProxy_NodeGroup::ckGetGroupID();}
operator CkGroupID () const { return ckGetGroupID(); }

    inline void setReductionClient(CkReductionClientFn fn,void *param=NULL) const
    { CProxy_NodeGroup::setReductionClient(fn,param); }
    inline void ckSetReductionClient(CkReductionClientFn fn,void *param=NULL) const
    { CProxy_NodeGroup::ckSetReductionClient(fn,param); }
    inline void ckSetReductionClient(CkCallback *cb) const
    { CProxy_NodeGroup::ckSetReductionClient(cb); }

    void ckDelegate(CkDelegateMgr *dTo,CkDelegateData *dPtr=NULL)
    {       CProxy_NodeGroup::ckDelegate(dTo,dPtr); }
    void ckUndelegate(void)
    {       CProxy_NodeGroup::ckUndelegate(); }
    void pup(PUP::er &p)
    {       CProxy_NodeGroup::pup(p); }
    void ckSetGroupID(CkGroupID g) {
      CProxy_NodeGroup::ckSetGroupID(g);
    }
    Cache* ckLocalBranch(void) const {
      return ckLocalBranch(ckGetGroupID());
    }
    static Cache* ckLocalBranch(CkGroupID gID) {
      return (Cache*)CkLocalNodeBranch(gID);
    }
/* DECLS: Cache(double ofub);
 */
    
    static CkGroupID ckNew(double ofub, const CkEntryOptions *impl_e_opts=NULL);
    CProxy_Cache(double ofub, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void updateOFUB(double ofub);
 */
    
    void updateOFUB(double ofub, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postConstraint(const set<constraint_t > &constraints);
 */
    
    void postConstraint(const set<constraint_t > &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postFinishedConstraint(const constraint_set_t &constraints);
 */
    
    void postFinishedConstraint(const constraint_set_t &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postDroppedConstraint(const set<constraint_set_t > &constraints);
 */
    
    void postDroppedConstraint(const set<constraint_set_t > &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void reportStat(void);
 */
    
    void reportStat(void);

};
PUPmarshall(CProxy_Cache)
/* ---------------- section proxy -------------- */
class CProxySection_Cache: public CProxySection_NodeGroup{
  public:
    typedef Cache local_t;
    typedef CkIndex_Cache index_t;
    typedef CProxy_Cache proxy_t;
    typedef CProxyElement_Cache element_t;
    typedef CProxySection_Cache section_t;

    CProxySection_Cache(void) {}
    CProxySection_Cache(const IrrGroup *g) : CProxySection_NodeGroup(g){  }
    CProxySection_Cache(const CkGroupID &_gid,const int *_pelist,int _npes,CK_DELCTOR_PARAM) : CProxySection_NodeGroup(_gid,_pelist,_npes,CK_DELCTOR_ARGS){  }
    CProxySection_Cache(const CkGroupID &_gid,const int *_pelist,int _npes) : CProxySection_NodeGroup(_gid,_pelist,_npes){  }
    CProxySection_Cache(int n,const CkGroupID *_gid, int const * const *_pelist,const int *_npes) : CProxySection_NodeGroup(n,_gid,_pelist,_npes){  }
    CProxySection_Cache(int n,const CkGroupID *_gid, int const * const *_pelist,const int *_npes,CK_DELCTOR_PARAM) : CProxySection_NodeGroup(n,_gid,_pelist,_npes,CK_DELCTOR_ARGS){  }

    int ckIsDelegated(void) const
    { return CProxySection_NodeGroup::ckIsDelegated(); }
    inline CkDelegateMgr *ckDelegatedTo(void) const
    { return CProxySection_NodeGroup::ckDelegatedTo(); }
    inline CkDelegateData *ckDelegatedPtr(void) const
    { return CProxySection_NodeGroup::ckDelegatedPtr(); }
    CkGroupID ckDelegatedIdx(void) const
    { return CProxySection_NodeGroup::ckDelegatedIdx(); }
inline void ckCheck(void) const {CProxySection_NodeGroup::ckCheck();}
CkChareID ckGetChareID(void) const
   {return CProxySection_NodeGroup::ckGetChareID();}
CkGroupID ckGetGroupID(void) const
   {return CProxySection_NodeGroup::ckGetGroupID();}
operator CkGroupID () const { return ckGetGroupID(); }

    inline void setReductionClient(CkReductionClientFn fn,void *param=NULL) const
    { CProxySection_NodeGroup::setReductionClient(fn,param); }
    inline void ckSetReductionClient(CkReductionClientFn fn,void *param=NULL) const
    { CProxySection_NodeGroup::ckSetReductionClient(fn,param); }
    inline void ckSetReductionClient(CkCallback *cb) const
    { CProxySection_NodeGroup::ckSetReductionClient(cb); }
inline int ckGetNumSections() const
{ return CProxySection_NodeGroup::ckGetNumSections(); }
inline CkSectionInfo &ckGetSectionInfo()
{ return CProxySection_NodeGroup::ckGetSectionInfo(); }
inline CkSectionID *ckGetSectionIDs()
{ return CProxySection_NodeGroup::ckGetSectionIDs(); }
inline CkSectionID &ckGetSectionID()
{ return CProxySection_NodeGroup::ckGetSectionID(); }
inline CkSectionID &ckGetSectionID(int i)
{ return CProxySection_NodeGroup::ckGetSectionID(i); }
inline CkGroupID ckGetGroupIDn(int i) const
{ return CProxySection_NodeGroup::ckGetGroupIDn(i); }
inline int *ckGetElements() const
{ return CProxySection_NodeGroup::ckGetElements(); }
inline int *ckGetElements(int i) const
{ return CProxySection_NodeGroup::ckGetElements(i); }
inline int ckGetNumElements() const
{ return CProxySection_NodeGroup::ckGetNumElements(); } 
inline int ckGetNumElements(int i) const
{ return CProxySection_NodeGroup::ckGetNumElements(i); }

    void ckDelegate(CkDelegateMgr *dTo,CkDelegateData *dPtr=NULL)
    {       CProxySection_NodeGroup::ckDelegate(dTo,dPtr); }
    void ckUndelegate(void)
    {       CProxySection_NodeGroup::ckUndelegate(); }
    void pup(PUP::er &p)
    {       CProxySection_NodeGroup::pup(p); }
    void ckSetGroupID(CkGroupID g) {
      CProxySection_NodeGroup::ckSetGroupID(g);
    }
    Cache* ckLocalBranch(void) const {
      return ckLocalBranch(ckGetGroupID());
    }
    static Cache* ckLocalBranch(CkGroupID gID) {
      return (Cache*)CkLocalNodeBranch(gID);
    }
/* DECLS: Cache(double ofub);
 */
    

/* DECLS: void updateOFUB(double ofub);
 */
    
    void updateOFUB(double ofub, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postConstraint(const set<constraint_t > &constraints);
 */
    
    void postConstraint(const set<constraint_t > &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postFinishedConstraint(const constraint_set_t &constraints);
 */
    
    void postFinishedConstraint(const constraint_set_t &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void postDroppedConstraint(const set<constraint_set_t > &constraints);
 */
    
    void postDroppedConstraint(const set<constraint_set_t > &constraints, const CkEntryOptions *impl_e_opts=NULL);

/* DECLS: void reportStat(void);
 */
    
    void reportStat(void);

};
PUPmarshall(CProxySection_Cache)
#define Cache_SDAG_CODE 
typedef CBaseT1<NodeGroup, CProxy_Cache>CBase_Cache;

/* DECLS: chare Slave: Chare{
Slave(double parent_cost, const constraint_t &constraints, int branch_col);
};
 */
 class Slave;
 class CkIndex_Slave;
 class CProxy_Slave;
/* --------------- index object ------------------ */
class CkIndex_Slave:public CkIndex_Chare{
  public:
    typedef Slave local_t;
    typedef CkIndex_Slave index_t;
    typedef CProxy_Slave proxy_t;
    typedef CProxy_Slave element_t;

    static int __idx;
    static void __register(const char *s, size_t size);
    /* DECLS: Slave(double parent_cost, const constraint_t &constraints, int branch_col);
     */
    // Entry point registration at startup
    
    static int reg_Slave_marshall1();
    // Entry point index lookup
    
    inline static int idx_Slave_marshall1() {
      static int epidx = reg_Slave_marshall1();
      return epidx;
    }

    
    static int ckNew(double parent_cost, const constraint_t &constraints, int branch_col) { return idx_Slave_marshall1(); }
    
    static void _call_Slave_marshall1(void* impl_msg, void* impl_obj);
    
    static void _call_sdag_Slave_marshall1(void* impl_msg, void* impl_obj);
    
    static int _callmarshall_Slave_marshall1(char* impl_buf, void* impl_obj_void);
    
    static void _marshallmessagepup_Slave_marshall1(PUP::er &p,void *msg);
};
/* --------------- element proxy ------------------ */
class CProxy_Slave:public CProxy_Chare{
  public:
    typedef Slave local_t;
    typedef CkIndex_Slave index_t;
    typedef CProxy_Slave proxy_t;
    typedef CProxy_Slave element_t;

    CProxy_Slave(void) {};
    CProxy_Slave(CkChareID __cid) : CProxy_Chare(__cid){  }
    CProxy_Slave(const Chare *c) : CProxy_Chare(c){  }

    int ckIsDelegated(void) const
    { return CProxy_Chare::ckIsDelegated(); }
    inline CkDelegateMgr *ckDelegatedTo(void) const
    { return CProxy_Chare::ckDelegatedTo(); }
    inline CkDelegateData *ckDelegatedPtr(void) const
    { return CProxy_Chare::ckDelegatedPtr(); }
    CkGroupID ckDelegatedIdx(void) const
    { return CProxy_Chare::ckDelegatedIdx(); }

    inline void ckCheck(void) const
    { CProxy_Chare::ckCheck(); }
    const CkChareID &ckGetChareID(void) const
    { return CProxy_Chare::ckGetChareID(); }
    operator const CkChareID &(void) const
    { return ckGetChareID(); }

    void ckDelegate(CkDelegateMgr *dTo,CkDelegateData *dPtr=NULL)
    {       CProxy_Chare::ckDelegate(dTo,dPtr); }
    void ckUndelegate(void)
    {       CProxy_Chare::ckUndelegate(); }
    void pup(PUP::er &p)
    {       CProxy_Chare::pup(p); }

    void ckSetChareID(const CkChareID &c)
    {      CProxy_Chare::ckSetChareID(c); }
    Slave *ckLocal(void) const
    { return (Slave *)CkLocalChare(&ckGetChareID()); }
/* DECLS: Slave(double parent_cost, const constraint_t &constraints, int branch_col);
 */
    static CkChareID ckNew(double parent_cost, const constraint_t &constraints, int branch_col, int onPE=CK_PE_ANY, const CkEntryOptions *impl_e_opts=NULL);
    static void ckNew(double parent_cost, const constraint_t &constraints, int branch_col, CkChareID* pcid, int onPE=CK_PE_ANY, const CkEntryOptions *impl_e_opts=NULL);
    CProxy_Slave(double parent_cost, const constraint_t &constraints, int branch_col, int onPE=CK_PE_ANY, const CkEntryOptions *impl_e_opts=NULL);

};
PUPmarshall(CProxy_Slave)
#define Slave_SDAG_CODE 
typedef CBaseT1<Chare, CProxy_Slave>CBase_Slave;













/* ---------------- method closures -------------- */
class Closure_Master {
  public:



    struct updateOFUB_3_closure;


    struct done_4_closure;


};

/* ---------------- method closures -------------- */
class Closure_Cache {
  public:


    struct updateOFUB_2_closure;


    struct postConstraint_3_closure;


    struct postFinishedConstraint_4_closure;


    struct postDroppedConstraint_5_closure;


    struct reportStat_6_closure;

};

/* ---------------- method closures -------------- */
class Closure_Slave {
  public:

};

extern void _registerilpprune(void);
extern "C" void CkRegisterMainModule(void);
#endif
