#ifndef _KALMAN_H_
#define _KALMAN_H_
//====================================
//				Include
//====================================


/**
Implementazione Kalman
@author Giovanni Micheli
*/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//====================================
//				Define

//file txt contenenti matrici necessarie per kalman
///\brief file contenente la matrice Q
#define MATRICEQ         "kalman_data/matriceQ.txt"
///\brief file contenente la matrice R
#define MATRICER         "kalman_data/matriceR.txt"
///\brief file contenente la matrice P
#define MATRICEP         "kalman_data/matriceP.txt"
///\brief file contenente il vettore xk
#define VETTOREXK         "kalman_data/vettoreXk.txt"
///\brief file contenente la tabella seno
#define TABELLASENO       "kalman_data/tabellaSeno.txt"

/**
\brief struttura che contiene dati post-calibrazione magnetometro
*/
typedef struct {
  float*   matriceQ;    ///< matrice di covarianza del rumore di processo (fissa)
  float*   matriceR;    ///< matrice di covarianza del rumore di misura (fissa)
  float*   matriceP;    ///< matrice di covarianza dell'errore (evolve)
  float*   matriceA;    ///< matrice Jacobiano delle derivate parziali di f rispetto allo stato
  float*   matriceAP;   ///< matrice A * P
  float*   matricePp;   ///< matrice di predizione della covarianza dell'errore
  float*   matriceH;    ///< matrice Jacobiano delle derivate parziali di h rispetto allo stato
  float*   matricePpHt; ///< matrice Pp * H'
  float*   appoggioInv; ///< matrice di appoggio per l'inversione = H*Pp*H'+R
  float    detInv;      ///< determinante utile per l'inversione
  float*   inversa;     ///< matrice inversa = (H*Pp*H'+R)^(-1)
  float*   xk;          ///< vettore contenente lo stato attuale del filtro di kalman
  float*   xkp;         ///< vettore contenente lo stato predetto del filtro di kalman
  float*   matriceK;    ///< matrice di guadagno di kalman
  float*   matriceKH;   ///< matrice K * H
  float*   vettoreDiff; ///< vettore dato da (Y'-xkp) di dimensione 3x1
  float*   tabellaSeno; ///< vettore valori del seno

} datiKalman;

///\brief puntatore alla struttura di tipo datiKalman che contiene i dati necessari per Kalman
datiKalman *dKalman;

//Inizializzo Tc, il tempo di campionamento
float tc = 0.25;

float kalman_input_c[2];
float kalman_input_m[3];

//====================================

//====================================
//		Functions declaration

//=====================================================================
//                          Principale
//=====================================================================
/**
Funzione che esegue un passo dell'algoritmo di Kalman
\brief avvio dell'algoritmo di Kalman
@param[in] dKalman puntatore che punta all'indirizzo di memoria della struct contenete i dati di kalman
@param[in] controllo[] vettore contenente gli ingressi di controllo
@param[in] misure[] vettore contenente le misure utili per la correzione
*/
int passoAlgoritmoKalman(datiKalman **dKalman, float controllo[], float misure[]);

//=====================================================================
//                          Fase iniziale
//=====================================================================
/**
Funzione che predispone i dati necessari per Kalman
\brief generazione della struttura contenente i dati per Kalman (datiKalman dkalman)
@param[in] dKalman puntatore che punta all'indirizzo di memoria della struct contenete i dati di kalman
*/
int inizializzaDatiKalman(datiKalman **dKalman);

/**
Funzione che libera lo spazio precedentemente riservato per i dati relativi a kalman
\brief eliminazione della struct datiKalman dKalman
@param[in] dKalman puntatore alla struct datiKalman
*/
void distruggiDatiKalman( datiKalman *dKalman );

//=====================================================================
//                       Operazioni su matrici
//=====================================================================
/**
Funzione per stampare a video la matrice.
\brief 	stampare a video la matrice
@param[in] testo carattere che identifica la matrice da stampare
@param[in] matricedastampare puntatore alla matrice da stampare
@param[in] righe righe della matrice da stampare
@param[in] colonne colonne della matrice da stampare
*/
void stampaMatrice(char testo,float *matricedastampare, int righe, int colonne);
//=====================================================================
//                        Calcoli matematici
//=====================================================================
/**
Funzione per calcolare sen(angolo) tramite una tabella (seno discretizzato con granularità di 0.2°)
\brief calcola il seno di un angolo
@param[in] dKalman puntatore che punta all'indirizzo di memoria della struct contenete i dati di kalman
@param[in] angolo angolo di cui si vuole calcolare il seno
@param[out] il seno risultante
*/
float calcolaSeno(datiKalman **dKalman, float angolo);


/**
Funzione per calcolare cos(angolo) tramite una tabella (seno discretizzato con granularità di 0.2°)
\brief calcola il coseno di un angolo
@param[in] dKalman puntatore che punta all'indirizzo di memoria della struct contenete i dati di kalman
@param[in] angolo angolo di cui si vuole calcolare il coseno
@param[out] il coseno risultante
*/
float calcolaCoseno(datiKalman **dKalman, float angolo);

//====================================
#endif
