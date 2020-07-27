/* Created by Language version: 7.5.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__STN
#define _nrn_initial _nrn_initial__STN
#define nrn_cur _nrn_cur__STN
#define _nrn_current _nrn_current__STN
#define nrn_jacob _nrn_jacob__STN
#define nrn_state _nrn_state__STN
#define _net_receive _net_receive__STN 
#define evaluate_fct evaluate_fct__STN 
#define states states__STN 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define ena _p[0]
#define ek _p[1]
#define tezao _p[2]
#define dbs _p[3]
#define gnabar _p[4]
#define gkdrbar _p[5]
#define gl _p[6]
#define el _p[7]
#define gcatbar _p[8]
#define gcalbar _p[9]
#define tau_d2 _p[10]
#define gkabar _p[11]
#define gkcabar _p[12]
#define ina _p[13]
#define ikD _p[14]
#define ikA _p[15]
#define ikAHP _p[16]
#define icaT _p[17]
#define icaL _p[18]
#define ilk _p[19]
#define idbs _p[20]
#define periodo _p[21]
#define h_inf _p[22]
#define tau_h _p[23]
#define m_inf _p[24]
#define tau_m _p[25]
#define n_inf _p[26]
#define tau_n _p[27]
#define p_inf _p[28]
#define q_inf _p[29]
#define tau_p _p[30]
#define tau_q _p[31]
#define eca _p[32]
#define c_inf _p[33]
#define tau_c _p[34]
#define d1_inf _p[35]
#define tau_d1 _p[36]
#define d2_inf _p[37]
#define a_inf _p[38]
#define tau_a _p[39]
#define b_inf _p[40]
#define tau_b _p[41]
#define r_inf _p[42]
#define m _p[43]
#define h _p[44]
#define n _p[45]
#define p _p[46]
#define q _p[47]
#define c _p[48]
#define d1 _p[49]
#define d2 _p[50]
#define cai _p[51]
#define a _p[52]
#define b _p[53]
#define r _p[54]
#define Dm _p[55]
#define Dh _p[56]
#define Dn _p[57]
#define Dp _p[58]
#define Dq _p[59]
#define Dc _p[60]
#define Dd1 _p[61]
#define Dd2 _p[62]
#define Dcai _p[63]
#define Da _p[64]
#define Db _p[65]
#define Dr _p[66]
#define v _p[67]
#define _g _p[68]
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_evaluate_fct(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_STN", _hoc_setdata,
 "evaluate_fct_STN", _hoc_evaluate_fct,
 0, 0
};
 /* declare global and static user variables */
#define acai acai_STN
 double acai = 0.00518;
#define bcai bcai_STN
 double bcai = 0.002;
#define con con_STN
 double con = 12.8392;
#define cao cao_STN
 double cao = 2000;
#define k_r k_r_STN
 double k_r = -0.08;
#define k_b k_b_STN
 double k_b = 7.5;
#define k_a k_a_STN
 double k_a = -14.7;
#define k_d2 k_d2_STN
 double k_d2 = 0.02;
#define k_d1 k_d1_STN
 double k_d1 = 7.5;
#define k_c k_c_STN
 double k_c = -5;
#define k_q k_q_STN
 double k_q = 5.8;
#define k_p k_p_STN
 double k_p = -6.7;
#define k_n k_n_STN
 double k_n = -14;
#define k_h k_h_STN
 double k_h = 6.4;
#define k_m k_m_STN
 double k_m = -8;
#define sig_b2 sig_b2_STN
 double sig_b2 = 10;
#define sig_b1 sig_b1_STN
 double sig_b1 = -30;
#define sig_a sig_a_STN
 double sig_a = -0.5;
#define sig_d12 sig_d12_STN
 double sig_d12 = 20;
#define sig_d11 sig_d11_STN
 double sig_d11 = -15;
#define sig_c2 sig_c2_STN
 double sig_c2 = 15;
#define sig_c1 sig_c1_STN
 double sig_c1 = -20;
#define sig_q2 sig_q2_STN
 double sig_q2 = 16;
#define sig_q1 sig_q1_STN
 double sig_q1 = -15;
#define sig_p2 sig_p2_STN
 double sig_p2 = 15;
#define sig_p1 sig_p1_STN
 double sig_p1 = -10;
#define sig_n2 sig_n2_STN
 double sig_n2 = 50;
#define sig_n1 sig_n1_STN
 double sig_n1 = -40;
#define sig_h2 sig_h2_STN
 double sig_h2 = 16;
#define sig_h1 sig_h1_STN
 double sig_h1 = -15;
#define sig_m sig_m_STN
 double sig_m = -0.7;
#define tau_r tau_r_STN
 double tau_r = 2;
#define theta_r theta_r_STN
 double theta_r = 0.17;
#define tht_b2 tht_b2_STN
 double tht_b2 = -40;
#define tht_b1 tht_b1_STN
 double tht_b1 = -60;
#define tht_a tht_a_STN
 double tht_a = -40;
#define tau_b1 tau_b1_STN
 double tau_b1 = 200;
#define tau_b0 tau_b0_STN
 double tau_b0 = 0;
#define tau_a1 tau_a1_STN
 double tau_a1 = 1;
#define tau_a0 tau_a0_STN
 double tau_a0 = 1;
#define theta_b theta_b_STN
 double theta_b = -90;
#define theta_a theta_a_STN
 double theta_a = -45;
#define tht_d12 tht_d12_STN
 double tht_d12 = -20;
#define tht_d11 tht_d11_STN
 double tht_d11 = -40;
#define tht_c2 tht_c2_STN
 double tht_c2 = -50;
#define tht_c1 tht_c1_STN
 double tht_c1 = -27;
#define tau_d11 tau_d11_STN
 double tau_d11 = 500;
#define tau_d10 tau_d10_STN
 double tau_d10 = 400;
#define tau_c1 tau_c1_STN
 double tau_c1 = 10;
#define tau_c0 tau_c0_STN
 double tau_c0 = 45;
#define theta_d2 theta_d2_STN
 double theta_d2 = 0.1;
#define theta_d1 theta_d1_STN
 double theta_d1 = -60;
#define theta_c theta_c_STN
 double theta_c = -30.6;
#define tht_q2 tht_q2_STN
 double tht_q2 = -50;
#define tht_q1 tht_q1_STN
 double tht_q1 = -50;
#define tht_p2 tht_p2_STN
 double tht_p2 = -102;
#define tht_p1 tht_p1_STN
 double tht_p1 = -27;
#define tau_q1 tau_q1_STN
 double tau_q1 = 400;
#define tau_q0 tau_q0_STN
 double tau_q0 = 0;
#define tau_p1 tau_p1_STN
 double tau_p1 = 0.33;
#define tau_p0 tau_p0_STN
 double tau_p0 = 5;
#define theta_q theta_q_STN
 double theta_q = -85;
#define theta_p theta_p_STN
 double theta_p = -56;
#define tht_n2 tht_n2_STN
 double tht_n2 = -40;
#define tht_n1 tht_n1_STN
 double tht_n1 = -40;
#define tau_n1 tau_n1_STN
 double tau_n1 = 11;
#define tau_n0 tau_n0_STN
 double tau_n0 = 0;
#define theta_n theta_n_STN
 double theta_n = -41;
#define tht_h2 tht_h2_STN
 double tht_h2 = -50;
#define tht_h1 tht_h1_STN
 double tht_h1 = -50;
#define tht_m tht_m_STN
 double tht_m = -53;
#define tau_h1 tau_h1_STN
 double tau_h1 = 24.5;
#define tau_h0 tau_h0_STN
 double tau_h0 = 0;
#define tau_m1 tau_m1_STN
 double tau_m1 = 3;
#define tau_m0 tau_m0_STN
 double tau_m0 = 0.2;
#define theta_h theta_h_STN
 double theta_h = -45.5;
#define theta_m theta_m_STN
 double theta_m = -40;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "con_STN", "mV",
 "theta_m_STN", "mV",
 "theta_h_STN", "mV",
 "k_m_STN", "mV",
 "k_h_STN", "mV",
 "tau_m0_STN", "ms",
 "tau_m1_STN", "ms",
 "tau_h0_STN", "ms",
 "tau_h1_STN", "ms",
 "tht_m_STN", "mV",
 "tht_h1_STN", "mV",
 "tht_h2_STN", "mV",
 "sig_m_STN", "mV",
 "sig_h1_STN", "mV",
 "sig_h2_STN", "mV",
 "theta_n_STN", "mV",
 "k_n_STN", "mV",
 "tau_n0_STN", "ms",
 "tau_n1_STN", "ms",
 "tht_n1_STN", "mV",
 "tht_n2_STN", "mV",
 "sig_n1_STN", "mV",
 "sig_n2_STN", "mV",
 "theta_p_STN", "mV",
 "theta_q_STN", "mV",
 "k_p_STN", "mV",
 "k_q_STN", "mV",
 "tau_p0_STN", "ms",
 "tau_p1_STN", "ms",
 "tau_q0_STN", "ms",
 "tau_q1_STN", "ms",
 "tht_p1_STN", "mV",
 "tht_p2_STN", "mV",
 "tht_q1_STN", "mV",
 "tht_q2_STN", "mV",
 "sig_p1_STN", "mV",
 "sig_p2_STN", "mV",
 "sig_q1_STN", "mV",
 "sig_q2_STN", "mV",
 "theta_c_STN", "mV",
 "theta_d1_STN", "mV",
 "theta_d2_STN", "mV",
 "k_c_STN", "mV",
 "k_d1_STN", "mV",
 "k_d2_STN", "mV",
 "tau_c0_STN", "ms",
 "tau_c1_STN", "ms",
 "tau_d10_STN", "ms",
 "tau_d11_STN", "ms",
 "tht_c1_STN", "mV",
 "tht_c2_STN", "mV",
 "tht_d11_STN", "mV",
 "tht_d12_STN", "mV",
 "sig_c1_STN", "mV",
 "sig_c2_STN", "mV",
 "sig_d11_STN", "mV",
 "sig_d12_STN", "mV",
 "theta_a_STN", "mV",
 "theta_b_STN", "mV",
 "k_a_STN", "mV",
 "k_b_STN", "mV",
 "tau_a0_STN", "ms",
 "tau_a1_STN", "ms",
 "tau_b0_STN", "ms",
 "tau_b1_STN", "ms",
 "tht_a_STN", "mV",
 "tht_b1_STN", "mV",
 "tht_b2_STN", "mV",
 "sig_a_STN", "mV",
 "sig_b1_STN", "mV",
 "sig_b2_STN", "mV",
 "theta_r_STN", "mV",
 "k_r_STN", "mV",
 "tau_r_STN", "ms",
 "acai_STN", "cm2/mA/ms",
 "bcai_STN", "1/ms",
 "ena_STN", "mV",
 "ek_STN", "mV",
 "tezao_STN", "ms",
 "gnabar_STN", "S/cm2",
 "gkdrbar_STN", "S/cm2",
 "gl_STN", "S/cm2",
 "el_STN", "mV",
 "gcatbar_STN", "S/cm2",
 "gcalbar_STN", "S/cm2",
 "tau_d2_STN", "ms",
 "gkabar_STN", "S/cm2",
 "gkcabar_STN", "S/cm2",
 "ina_STN", "mA/cm2",
 "ikD_STN", "mA/cm2",
 "ikA_STN", "mA/cm2",
 "ikAHP_STN", "mA/cm2",
 "icaT_STN", "mA/cm2",
 "icaL_STN", "mA/cm2",
 "ilk_STN", "mA/cm2",
 "idbs_STN", "mA/cm2",
 "periodo_STN", "ms",
 "tau_h_STN", "ms",
 "tau_m_STN", "ms",
 "tau_n_STN", "ms",
 "tau_p_STN", "ms",
 "tau_q_STN", "ms",
 "eca_STN", "mV",
 "tau_c_STN", "ms",
 "tau_d1_STN", "ms",
 "tau_a_STN", "ms",
 "tau_b_STN", "ms",
 0,0
};
 static double a0 = 0;
 static double b0 = 0;
 static double cai0 = 0;
 static double c0 = 0;
 static double delta_t = 0.01;
 static double d20 = 0;
 static double d10 = 0;
 static double h0 = 0;
 static double m0 = 0;
 static double n0 = 0;
 static double p0 = 0;
 static double q0 = 0;
 static double r0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "cao_STN", &cao_STN,
 "con_STN", &con_STN,
 "theta_m_STN", &theta_m_STN,
 "theta_h_STN", &theta_h_STN,
 "k_m_STN", &k_m_STN,
 "k_h_STN", &k_h_STN,
 "tau_m0_STN", &tau_m0_STN,
 "tau_m1_STN", &tau_m1_STN,
 "tau_h0_STN", &tau_h0_STN,
 "tau_h1_STN", &tau_h1_STN,
 "tht_m_STN", &tht_m_STN,
 "tht_h1_STN", &tht_h1_STN,
 "tht_h2_STN", &tht_h2_STN,
 "sig_m_STN", &sig_m_STN,
 "sig_h1_STN", &sig_h1_STN,
 "sig_h2_STN", &sig_h2_STN,
 "theta_n_STN", &theta_n_STN,
 "k_n_STN", &k_n_STN,
 "tau_n0_STN", &tau_n0_STN,
 "tau_n1_STN", &tau_n1_STN,
 "tht_n1_STN", &tht_n1_STN,
 "tht_n2_STN", &tht_n2_STN,
 "sig_n1_STN", &sig_n1_STN,
 "sig_n2_STN", &sig_n2_STN,
 "theta_p_STN", &theta_p_STN,
 "theta_q_STN", &theta_q_STN,
 "k_p_STN", &k_p_STN,
 "k_q_STN", &k_q_STN,
 "tau_p0_STN", &tau_p0_STN,
 "tau_p1_STN", &tau_p1_STN,
 "tau_q0_STN", &tau_q0_STN,
 "tau_q1_STN", &tau_q1_STN,
 "tht_p1_STN", &tht_p1_STN,
 "tht_p2_STN", &tht_p2_STN,
 "tht_q1_STN", &tht_q1_STN,
 "tht_q2_STN", &tht_q2_STN,
 "sig_p1_STN", &sig_p1_STN,
 "sig_p2_STN", &sig_p2_STN,
 "sig_q1_STN", &sig_q1_STN,
 "sig_q2_STN", &sig_q2_STN,
 "theta_c_STN", &theta_c_STN,
 "theta_d1_STN", &theta_d1_STN,
 "theta_d2_STN", &theta_d2_STN,
 "k_c_STN", &k_c_STN,
 "k_d1_STN", &k_d1_STN,
 "k_d2_STN", &k_d2_STN,
 "tau_c0_STN", &tau_c0_STN,
 "tau_c1_STN", &tau_c1_STN,
 "tau_d10_STN", &tau_d10_STN,
 "tau_d11_STN", &tau_d11_STN,
 "tht_c1_STN", &tht_c1_STN,
 "tht_c2_STN", &tht_c2_STN,
 "tht_d11_STN", &tht_d11_STN,
 "tht_d12_STN", &tht_d12_STN,
 "sig_c1_STN", &sig_c1_STN,
 "sig_c2_STN", &sig_c2_STN,
 "sig_d11_STN", &sig_d11_STN,
 "sig_d12_STN", &sig_d12_STN,
 "theta_a_STN", &theta_a_STN,
 "theta_b_STN", &theta_b_STN,
 "k_a_STN", &k_a_STN,
 "k_b_STN", &k_b_STN,
 "tau_a0_STN", &tau_a0_STN,
 "tau_a1_STN", &tau_a1_STN,
 "tau_b0_STN", &tau_b0_STN,
 "tau_b1_STN", &tau_b1_STN,
 "tht_a_STN", &tht_a_STN,
 "tht_b1_STN", &tht_b1_STN,
 "tht_b2_STN", &tht_b2_STN,
 "sig_a_STN", &sig_a_STN,
 "sig_b1_STN", &sig_b1_STN,
 "sig_b2_STN", &sig_b2_STN,
 "theta_r_STN", &theta_r_STN,
 "k_r_STN", &k_r_STN,
 "tau_r_STN", &tau_r_STN,
 "acai_STN", &acai_STN,
 "bcai_STN", &bcai_STN,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[0]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.5.0",
"STN",
 "ena_STN",
 "ek_STN",
 "tezao_STN",
 "dbs_STN",
 "gnabar_STN",
 "gkdrbar_STN",
 "gl_STN",
 "el_STN",
 "gcatbar_STN",
 "gcalbar_STN",
 "tau_d2_STN",
 "gkabar_STN",
 "gkcabar_STN",
 0,
 "ina_STN",
 "ikD_STN",
 "ikA_STN",
 "ikAHP_STN",
 "icaT_STN",
 "icaL_STN",
 "ilk_STN",
 "idbs_STN",
 "periodo_STN",
 "h_inf_STN",
 "tau_h_STN",
 "m_inf_STN",
 "tau_m_STN",
 "n_inf_STN",
 "tau_n_STN",
 "p_inf_STN",
 "q_inf_STN",
 "tau_p_STN",
 "tau_q_STN",
 "eca_STN",
 "c_inf_STN",
 "tau_c_STN",
 "d1_inf_STN",
 "tau_d1_STN",
 "d2_inf_STN",
 "a_inf_STN",
 "tau_a_STN",
 "b_inf_STN",
 "tau_b_STN",
 "r_inf_STN",
 0,
 "m_STN",
 "h_STN",
 "n_STN",
 "p_STN",
 "q_STN",
 "c_STN",
 "d1_STN",
 "d2_STN",
 "cai_STN",
 "a_STN",
 "b_STN",
 "r_STN",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 69, _prop);
 	/*initialize range parameters*/
 	ena = 60;
 	ek = -90;
 	tezao = 7.6923;
 	dbs = 0;
 	gnabar = 0.049;
 	gkdrbar = 0.057;
 	gl = 0.00035;
 	el = -60;
 	gcatbar = 0.005;
 	gcalbar = 0.015;
 	tau_d2 = 130;
 	gkabar = 0.005;
 	gkcabar = 0.001;
 	_prop->param = _p;
 	_prop->param_size = 69;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 1, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _STN_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 69, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 STN /home/jhielson/catkin_ws/src/neural_model/scripts/x86_64/STN.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "STN ion channels for single compartment model";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int evaluate_fct(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[12], _dlist1[12];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   evaluate_fct ( _threadargscomma_ v ) ;
   Dh = ( h_inf - h ) / tau_h ;
   Dm = ( m_inf - m ) / tau_m ;
   Dn = ( n_inf - n ) / tau_n ;
   Dp = ( p_inf - p ) / tau_p ;
   Dq = ( q_inf - q ) / tau_q ;
   Dc = ( c_inf - c ) / tau_c ;
   Dd1 = ( d1_inf - d1 ) / tau_d1 ;
   Dd2 = ( d2_inf - d2 ) / tau_d2 ;
   Dcai = - acai * ( icaL + icaT ) - bcai * cai ;
   Da = ( a_inf - a ) / tau_a ;
   Db = ( b_inf - b ) / tau_b ;
   Dr = ( r_inf - r ) / tau_r ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 evaluate_fct ( _threadargscomma_ v ) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_h )) ;
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_m )) ;
 Dn = Dn  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_n )) ;
 Dp = Dp  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_p )) ;
 Dq = Dq  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_q )) ;
 Dc = Dc  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_c )) ;
 Dd1 = Dd1  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_d1 )) ;
 Dd2 = Dd2  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_d2 )) ;
 Dcai = Dcai  / (1. - dt*( ( - ( bcai )*( 1.0 ) ) )) ;
 Da = Da  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_a )) ;
 Db = Db  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_b )) ;
 Dr = Dr  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_r )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   evaluate_fct ( _threadargscomma_ v ) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_h)))*(- ( ( ( h_inf ) ) / tau_h ) / ( ( ( ( - 1.0 ) ) ) / tau_h ) - h) ;
    m = m + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_m)))*(- ( ( ( m_inf ) ) / tau_m ) / ( ( ( ( - 1.0 ) ) ) / tau_m ) - m) ;
    n = n + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_n)))*(- ( ( ( n_inf ) ) / tau_n ) / ( ( ( ( - 1.0 ) ) ) / tau_n ) - n) ;
    p = p + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_p)))*(- ( ( ( p_inf ) ) / tau_p ) / ( ( ( ( - 1.0 ) ) ) / tau_p ) - p) ;
    q = q + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_q)))*(- ( ( ( q_inf ) ) / tau_q ) / ( ( ( ( - 1.0 ) ) ) / tau_q ) - q) ;
    c = c + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_c)))*(- ( ( ( c_inf ) ) / tau_c ) / ( ( ( ( - 1.0 ) ) ) / tau_c ) - c) ;
    d1 = d1 + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_d1)))*(- ( ( ( d1_inf ) ) / tau_d1 ) / ( ( ( ( - 1.0 ) ) ) / tau_d1 ) - d1) ;
    d2 = d2 + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_d2)))*(- ( ( ( d2_inf ) ) / tau_d2 ) / ( ( ( ( - 1.0 ) ) ) / tau_d2 ) - d2) ;
    cai = cai + (1. - exp(dt*(( - ( bcai )*( 1.0 ) ))))*(- ( ( - acai )*( ( icaL + icaT ) ) ) / ( ( - ( bcai )*( 1.0 ) ) ) - cai) ;
    a = a + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_a)))*(- ( ( ( a_inf ) ) / tau_a ) / ( ( ( ( - 1.0 ) ) ) / tau_a ) - a) ;
    b = b + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_b)))*(- ( ( ( b_inf ) ) / tau_b ) / ( ( ( ( - 1.0 ) ) ) / tau_b ) - b) ;
    r = r + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_r)))*(- ( ( ( r_inf ) ) / tau_r ) / ( ( ( ( - 1.0 ) ) ) / tau_r ) - r) ;
   }
  return 0;
}
 
static int  evaluate_fct ( _threadargsprotocomma_ double _lv ) {
   h_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_h ) / k_h ) ) ;
   m_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_m ) / k_m ) ) ;
   tau_h = tau_h0 + tau_h1 / ( exp ( - ( _lv - tht_h1 ) / sig_h1 ) + exp ( - ( _lv - tht_h2 ) / sig_h2 ) ) ;
   tau_m = tau_m0 + tau_m1 / ( 1.0 + exp ( - ( _lv - tht_m ) / sig_m ) ) ;
   n_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_n ) / k_n ) ) ;
   tau_n = tau_n0 + tau_n1 / ( exp ( - ( _lv - tht_n1 ) / sig_n1 ) + exp ( - ( _lv - tht_n2 ) / sig_n2 ) ) ;
   p_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_p ) / k_p ) ) ;
   q_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_q ) / k_q ) ) ;
   tau_p = tau_p0 + tau_p1 / ( exp ( - ( _lv - tht_p1 ) / sig_p1 ) + exp ( - ( _lv - tht_p2 ) / sig_p2 ) ) ;
   tau_q = tau_q0 + tau_q1 / ( exp ( - ( _lv - tht_q1 ) / sig_q1 ) + exp ( - ( _lv - tht_q2 ) / sig_q2 ) ) ;
   c_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_c ) / k_c ) ) ;
   d1_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_d1 ) / k_d1 ) ) ;
   d2_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_d2 ) / k_d2 ) ) ;
   tau_c = tau_c0 + tau_c1 / ( exp ( - ( _lv - tht_c1 ) / sig_c1 ) + exp ( - ( _lv - tht_c2 ) / sig_c2 ) ) ;
   tau_d1 = tau_d10 + tau_d11 / ( exp ( - ( _lv - tht_d11 ) / sig_d11 ) + exp ( - ( _lv - tht_d12 ) / sig_d12 ) ) ;
   a_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_a ) / k_a ) ) ;
   b_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_b ) / k_b ) ) ;
   tau_a = tau_a0 + tau_a1 / ( 1.0 + exp ( - ( _lv - tht_a ) / sig_a ) ) ;
   tau_b = tau_b0 + tau_b1 / ( exp ( - ( _lv - tht_b1 ) / sig_b1 ) + exp ( - ( _lv - tht_b2 ) / sig_b2 ) ) ;
   r_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_r ) / k_r ) ) ;
    return 0; }
 
static void _hoc_evaluate_fct(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 evaluate_fct ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 12;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 12; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  a = a0;
  b = b0;
  c = c0;
  cai = cai0;
  d2 = d20;
  d1 = d10;
  h = h0;
  m = m0;
  n = n0;
  p = p0;
  q = q0;
  r = r0;
 {
   evaluate_fct ( _threadargscomma_ v ) ;
   m = m_inf ;
   h = h_inf ;
   n = n_inf ;
   p = p_inf ;
   q = q_inf ;
   c = c_inf ;
   d1 = d1_inf ;
   d2 = d2_inf ;
   a = a_inf ;
   b = b_inf ;
   r = r_inf ;
   cai = 0.005 ;
   periodo = 0.0 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   eca = con * log ( cao / cai ) ;
   ina = gnabar * m * m * m * h * ( v - ena ) ;
   ikD = gkdrbar * pow( n , 4.0 ) * ( v - ek ) ;
   ikA = gkabar * a * a * b * ( v - ek ) ;
   ikAHP = gkcabar * r * r * ( v - ek ) ;
   icaT = gcatbar * p * p * q * ( v - eca ) ;
   icaL = gcalbar * c * c * d1 * d2 * ( v - eca ) ;
   ilk = gl * ( v - el ) ;
   if ( t >= periodo + tezao ) {
     periodo = periodo + tezao ;
     }
   if ( t >= periodo  && t <= periodo + 0.3 ) {
     idbs = - 0.3 * dbs ;
     }
   else {
     idbs = 0.0 ;
     }
   }
 _current += ilk;
 _current += icaT;
 _current += icaL;
 _current += ikD;
 _current += ikA;
 _current += ikAHP;
 _current += ina;
 _current += idbs;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(h) - _p;  _dlist1[0] = &(Dh) - _p;
 _slist1[1] = &(m) - _p;  _dlist1[1] = &(Dm) - _p;
 _slist1[2] = &(n) - _p;  _dlist1[2] = &(Dn) - _p;
 _slist1[3] = &(p) - _p;  _dlist1[3] = &(Dp) - _p;
 _slist1[4] = &(q) - _p;  _dlist1[4] = &(Dq) - _p;
 _slist1[5] = &(c) - _p;  _dlist1[5] = &(Dc) - _p;
 _slist1[6] = &(d1) - _p;  _dlist1[6] = &(Dd1) - _p;
 _slist1[7] = &(d2) - _p;  _dlist1[7] = &(Dd2) - _p;
 _slist1[8] = &(cai) - _p;  _dlist1[8] = &(Dcai) - _p;
 _slist1[9] = &(a) - _p;  _dlist1[9] = &(Da) - _p;
 _slist1[10] = &(b) - _p;  _dlist1[10] = &(Db) - _p;
 _slist1[11] = &(r) - _p;  _dlist1[11] = &(Dr) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
