//=====================================================================
//                  Filtro di Kalman ESTESO
//=====================================================================

/*Il filtro di Kalman esteso è una versione non lineare del filtro di Kalman,
utilizzato per la stima dello stato dell'uniciclo (che ha almeno 3 componenti, dx, dy, dtheta).
La caratteristica principale è che si linearizza ogni volta lo stato intorno alla stima precedente (passo k-1).

Si può pensare di avere uno stato composto da dx = cos(theta)*u(x) | dy = sin(theta)*u(y) | dtheta = w (velocità angolare)

Se aggiungo il rumore avrò

dx = cos(theta)*u(x) + v(x)

dy = sin(theta)*u(y) + v(y)

dtheta = w (velocità angolare)  + g


dove v(x) | v(y) | g (gamma) sono rumori bianchi, indipendenti e a media nulla tranne gamma che è a media non nulla.
quindi gamma può essere scomposto cosi

dtheta = w + v(theta) + g~ ovvero in una parte casuale e una parte di media diversa da 0.

    La funzione f descrive l'evoluzione dello stato; in forma matriciale:

	x_(k+1) = x_k + Tc*cos(theta_k)*ur_k
	y_(k+1) = y_k + Tc*sin(theta_k)*ur_k
	theta_(k+1) = theta_k + Tc*ua_k

	dove ur_k è l'ingresso relativo alla velocità lineare
	e ua_k è l'ingresso relativo alla velocità angolare

    A è la matrice Jacobiano delle derivate parziali di f rispetto allo stato:

	[1][0][-sen(theta_k)*Tc*ur_k]
	[0][1][cos(theta_k)*Tc*ur_k]
	[0][0][1]

    La funzione h descrive la relazione tra le misure e lo stato

    H è la matrice Jacobiano delle derivate parziali di h rispetto allo stato

    [1][0][0]
    [0][1][0]
    [0][0][1]

    Altri elementi che ho sono:
    -tempo di campionamento Tc di 200ms
    -Q matrice covarianza del rumore di processo (fissa)
    -R matrice di covarianza del rumore di misura(fissa)
    -U contiene gli ingressi di controllo
    -X contiene i valori dello stato ad ogni istante(x,y e theta)
    -Y contiene le misure (x e y odometria e theta magneto)
    -P matrice covarianza dell'errore (evolve)


	La matrice di covarianza P  e Q sono quadrate cosiccome gli jacobiani A e H.

*
*
* */

/*File .c per calcolare la predizione e la correzione secondo il filtro di kalman

	* * caratteristiche:
*
* 		-passaggio per puntatori
* 		-bisogna gestire dall'esterno il relativo controllo semaforico all'accesso dei dati condivisi quali
* 		 stima e covarianza
* 		-l'inversione di matrice potrebbe bloccarsi a causa del fatto che la matrice è fatta
* 		 di interi non positivi (<=0). Una possibile soluzione può essere lo skip del passo di correzione.
*/

#include "kalman.h"


/*int mainne()
//int main(int argc, char *argv[])
{

    //Inizializzo la struttura che contiene i dati per kalman
    inizializzaDatiKalman(&dKalman);

    //prendo U (ingressi di controllo)
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    //Gli ingressi di controllo sono già disponibili, adesso li simulo
    FILE *fmatriceU;
    fmatriceU=fopen("disponibili/matriceU.txt","r");
    float* matriceU;
    matriceU = (float*)malloc(sizeof(float)* 2 * 449);
    int riga=0;
    while(fscanf(fmatriceU,"%f %f\n",&matriceU[(riga*2)], &matriceU[(riga*2)+1])!=EOF) {
	riga++;
    }
    // chiusura file
    fclose(fmatriceU);
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    //prendo Y
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    //Come per U anche Y lo simulo
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    FILE *fmatriceY;
    fmatriceY=fopen("disponibili/matriceY.txt","r");
    float* matriceY;
    matriceY = (float*)malloc(sizeof(float)* 3 * 449);
    riga=0;
    while(fscanf(fmatriceY,"%f %f %f\n",&matriceY[(riga*3)], &matriceY[(riga*3)+1], &matriceY[(riga*3)+2])!=EOF) {
	riga++;
    }
    // chiusura file
    fclose(fmatriceY);
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx



    //apro un file per salvare l'output di kalman
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    FILE *foutput;
    foutput=fopen("disponibili/output.txt","w");
    //xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx






    //stampaMatrice('Q',dKalman->matriceQ, 3, 3);
    //stampaMatrice('x',dKalman->xk, 1, 3);
    //stampaMatrice('A',dKalman->matriceA, 3, 3);
    //stampaMatrice('x',dKalman->matriceH , 3, 3);

    int i;

    for (i=0;i<449;i++) {
	kalman_input_m[0] = matriceY[i*3];
	kalman_input_m[1] = matriceY[i*3+1];
	kalman_input_m[2] = matriceY[i*3+2];
	kalman_input_c[0] = matriceU[i*2];
	kalman_input_c[1] = matriceU[i*2+1];

	//fprintf(foutput,"%f %f\n", cos((dKalman)->xk[2]), sin((dKalman)->xk[2]));


	passoAlgoritmoKalman(&dKalman, kalman_input_c, kalman_input_m);

	fprintf(foutput,"%f %f %f\n", dKalman->xkp[0], dKalman->xkp[1], dKalman->xkp[2]);

    }

    // -misure ok
    // -controlli ok
    //

    stampaMatrice('P',dKalman->matriceP, 3, 3);
    //printf("%f", calcolaCoseno(&dKalman, 2));

    // chiusura file
    fclose(foutput);

    distruggiDatiKalman(dKalman);




    return 0;
}
*/


//=====================================================================
//                          Principale
//=====================================================================

//esegue un passo dell'algoritmo di Kalman
int passoAlgoritmoKalman(datiKalman **dKalman, float controllo[], float misure[]) {
    
    //l'algoritmo è suddiviso in 2 fasi:predizione e correzione
    //printf("%f %f %f\n", (*dKalman)->xkp[0], (*dKalman)->xkp[1], (*dKalman)->xkp[2]);

	    //==========================Predizione==============================


    //predizione dello stato legata all'evoluzione dell'uniciclo
    // prende come input il vettore della velocità lineare e angolare
    //(*dKalman)->xkp[0] = (*dKalman)->xk[0] + tc*controllo[0]*calcolaCoseno(dKalman,(*dKalman)->xk[2]);
    //(*dKalman)->xkp[1] = (*dKalman)->xk[1] + tc*controllo[0]*calcolaSeno(dKalman, (*dKalman)->xk[2]);
    (*dKalman)->xkp[0] = (*dKalman)->xk[0] + tc*controllo[0]*cos((*dKalman)->xk[2]);
    (*dKalman)->xkp[1] = (*dKalman)->xk[1] + tc*controllo[0]*sin((*dKalman)->xk[2]);
    
    //riporto tra -pi e +pi
    float thetaPTemp = (*dKalman)->xk[2] + tc*controllo[1];

    if (thetaPTemp > M_PI) {
	thetaPTemp -= (2 * M_PI);
    }
    if (thetaPTemp<-M_PI) {
	thetaPTemp += (2 * M_PI);
    }

    (*dKalman)->xkp[2] = thetaPTemp;


    //calcolo della matrice A
    //(*dKalman)->matriceA[2] = -1*tc*controllo[0]*calcolaSeno(dKalman,(*dKalman)->xk[2]);
    //(*dKalman)->matriceA[5] =  1*tc*controllo[0]*calcolaCoseno(dKalman,(*dKalman)->xk[2]);
    (*dKalman)->matriceA[2] = -1*tc*controllo[0]*sin((*dKalman)->xk[2]);
    (*dKalman)->matriceA[5] =  1*tc*controllo[0]*cos((*dKalman)->xk[2]);


    //calcolo matrice A*P
    //le moltiplicazioni con valori nulli non le faccio
    //elementi nulli in A in posizione 1,3,6,7
    (*dKalman)->matriceAP[0] =   (*dKalman)->matriceA[0] * (*dKalman)->matriceP[0] +
				(*dKalman)->matriceA[2] * (*dKalman)->matriceP[6];
			      //(*dKalman)->matriceA[1] * (*dKalman)->matriceP[3]

    (*dKalman)->matriceAP[1] =   (*dKalman)->matriceA[0] * (*dKalman)->matriceP[1] +
				(*dKalman)->matriceA[2] * (*dKalman)->matriceP[7];
			      //(*dKalman)->matriceA[1] * (*dKalman)->matriceP[4]

    (*dKalman)->matriceAP[2] =   (*dKalman)->matriceA[0] * (*dKalman)->matriceP[2] +
				(*dKalman)->matriceA[2] * (*dKalman)->matriceP[8];
			      //(*dKalman)->matriceA[1] * (*dKalman)->matriceP[5]

    (*dKalman)->matriceAP[3] =   (*dKalman)->matriceA[4] * (*dKalman)->matriceP[3] +
				(*dKalman)->matriceA[5] * (*dKalman)->matriceP[6];
			      //(*dKalman)->matriceA[3] * (*dKalman)->matriceP[0]

    (*dKalman)->matriceAP[4] =   (*dKalman)->matriceA[4] * (*dKalman)->matriceP[4] +
				(*dKalman)->matriceA[5] * (*dKalman)->matriceP[7];
			      //(*dKalman)->matriceA[3] * (*dKalman)->matriceP[1]

    (*dKalman)->matriceAP[5] =   (*dKalman)->matriceA[4] * (*dKalman)->matriceP[5] +
				(*dKalman)->matriceA[5] * (*dKalman)->matriceP[8];
			      //(*dKalman)->matriceA[3] * (*dKalman)->matriceP[2]

    (*dKalman)->matriceAP[6] =   (*dKalman)->matriceA[8] * (*dKalman)->matriceP[6];
			      //(*dKalman)->matriceA[6] * (*dKalman)->matriceP[0]
			      //(*dKalman)->matriceA[7] * (*dKalman)->matriceP[3]

    (*dKalman)->matriceAP[7] =   (*dKalman)->matriceA[8] * (*dKalman)->matriceP[7];
			      //(*dKalman)->matriceA[6] * (*dKalman)->matriceP[1]
			      //(*dKalman)->matriceA[7] * (*dKalman)->matriceP[4]

    (*dKalman)->matriceAP[8] =   (*dKalman)->matriceA[8] * (*dKalman)->matriceP[8];
			      //(*dKalman)->matriceA[6] * (*dKalman)->matriceP[2]
			      //(*dKalman)->matriceA[7] * (*dKalman)->matriceP[5]


    //predizione della covarianza dell'errore, calcolo matrice Pp
    //le moltiplicazioni con valori nulli non le faccio
    //elementi nulli in A in posizione 1,3,6,7
    (*dKalman)->matricePp[0] =  (*dKalman)->matriceAP[0] * (*dKalman)->matriceA[0] +
				(*dKalman)->matriceAP[2] * (*dKalman)->matriceA[2] +
				(*dKalman)->matriceQ[0];
			      //(*dKalman)->matriceAP[1] * (*dKalman)->matriceA[1]

    (*dKalman)->matricePp[1] =  (*dKalman)->matriceAP[1] * (*dKalman)->matriceA[4] +
				(*dKalman)->matriceAP[2] * (*dKalman)->matriceA[5] +
				(*dKalman)->matriceQ[1];
			      //(*dKalman)->matriceAP[0] * (*dKalman)->matriceA[3]

    (*dKalman)->matricePp[2] =  (*dKalman)->matriceAP[2] * (*dKalman)->matriceA[8] +
				(*dKalman)->matriceQ[2];
			      //(*dKalman)->matriceAP[0] * (*dKalman)->matriceA[6]
			      //(*dKalman)->matriceAP[1] * (*dKalman)->matriceA[7]

    (*dKalman)->matricePp[3] =  (*dKalman)->matriceAP[3] * (*dKalman)->matriceA[0] +
				(*dKalman)->matriceAP[5] * (*dKalman)->matriceA[2] +
				(*dKalman)->matriceQ[3];
			      //(*dKalman)->matriceAP[4] * (*dKalman)->matriceA[1]

    (*dKalman)->matricePp[4] =  (*dKalman)->matriceAP[4] * (*dKalman)->matriceA[4] +
				(*dKalman)->matriceAP[5] * (*dKalman)->matriceA[5] +
				(*dKalman)->matriceQ[4];
			      //(*dKalman)->matriceAP[3] * (*dKalman)->matriceA[3]

    (*dKalman)->matricePp[5] =  (*dKalman)->matriceAP[5] * (*dKalman)->matriceA[8] +
				(*dKalman)->matriceQ[5];
			      //(*dKalman)->matriceAP[3] * (*dKalman)->matriceA[6]
			      //(*dKalman)->matriceAP[4] * (*dKalman)->matriceA[7]

    (*dKalman)->matricePp[6] =  (*dKalman)->matriceAP[6] * (*dKalman)->matriceA[0] +
				(*dKalman)->matriceAP[8] * (*dKalman)->matriceA[2] +
				(*dKalman)->matriceQ[6];
			      //(*dKalman)->matriceAP[7] * (*dKalman)->matriceA[1]

    (*dKalman)->matricePp[7] =  (*dKalman)->matriceAP[7] * (*dKalman)->matriceA[4] +
				(*dKalman)->matriceAP[8] * (*dKalman)->matriceA[5] +
				(*dKalman)->matriceQ[7];
			      //(*dKalman)->matriceAP[6] * (*dKalman)->matriceA[3]

    (*dKalman)->matricePp[8] =  (*dKalman)->matriceAP[8] * (*dKalman)->matriceA[8] +
				(*dKalman)->matriceQ[8];
			      //(*dKalman)->matriceAP[6] * (*dKalman)->matriceA[6]
			      //(*dKalman)->matriceAP[7] * (*dKalman)->matriceA[7]




		//==========================Correzione==============================


    //calcolo matrice Pp * H'
    //le moltiplicazioni con valori nulli non le faccio
    //elementi nulli in H in posizione 1,2,3,5,6,7
    (*dKalman)->matricePpHt[0] =  (*dKalman)->matricePp[0] * (*dKalman)->matriceH[0];
				//(*dKalman)->matricePp[1] * (*dKalman)->matriceH[1]
				//(*dKalman)->matricePp[2] * (*dKalman)->matriceH[2]

    (*dKalman)->matricePpHt[1] =  (*dKalman)->matricePp[1] * (*dKalman)->matriceH[4];
				//(*dKalman)->matricePp[0] * (*dKalman)->matriceH[3]
				//(*dKalman)->matricePp[2] * (*dKalman)->matriceH[5]

    (*dKalman)->matricePpHt[2] =  (*dKalman)->matricePp[2] * (*dKalman)->matriceH[8];
				//(*dKalman)->matricePp[0] * (*dKalman)->matriceH[6]
				//(*dKalman)->matricePp[1] * (*dKalman)->matriceH[7]

    (*dKalman)->matricePpHt[3] =  (*dKalman)->matricePp[3] * (*dKalman)->matriceH[0];
				//(*dKalman)->matricePp[4] * (*dKalman)->matriceH[1]
				//(*dKalman)->matricePp[5] * (*dKalman)->matriceH[2]

    (*dKalman)->matricePpHt[4] =  (*dKalman)->matricePp[4] * (*dKalman)->matriceH[4];
				//(*dKalman)->matricePp[3] * (*dKalman)->matriceH[3]
				//(*dKalman)->matricePp[5] * (*dKalman)->matriceH[5]

    (*dKalman)->matricePpHt[5] =  (*dKalman)->matricePp[5] * (*dKalman)->matriceH[8];
				//(*dKalman)->matricePp[3] * (*dKalman)->matriceH[6]
				//(*dKalman)->matricePp[4] * (*dKalman)->matriceH[7]

    (*dKalman)->matricePpHt[6] =  (*dKalman)->matricePp[6] * (*dKalman)->matriceH[0];
				//(*dKalman)->matricePp[7] * (*dKalman)->matriceH[1]
				//(*dKalman)->matricePp[8] * (*dKalman)->matriceH[2]

    (*dKalman)->matricePpHt[7] =  (*dKalman)->matricePp[7] * (*dKalman)->matriceH[4];
				//(*dKalman)->matricePp[6] * (*dKalman)->matriceH[3]
				//(*dKalman)->matricePp[8] * (*dKalman)->matriceH[5]

    (*dKalman)->matricePpHt[8] =  (*dKalman)->matricePp[8] * (*dKalman)->matriceH[8];
				//(*dKalman)->matricePp[6] * (*dKalman)->matriceH[6]
				//(*dKalman)->matricePp[7] * (*dKalman)->matriceH[7]


    //calcolo matrice appoggioInv =  = H*Pp*H'+R
    //le moltiplicazioni con valori nulli non le faccio
    //elementi nulli in H in posizione 1,2,3,5,6,7
    //in più R (nel nostro caso) si può supporre sempre diagonale quindi
    //elementi nulli in R in posizione 1,2,3,5,6,7
    (*dKalman)->appoggioInv[0] =  (*dKalman)->matriceH[0] * (*dKalman)->matricePpHt[0] +
				  (*dKalman)->matriceR[0];
				//(*dKalman)->matriceH[1] * (*dKalman)->matricePpHt[3]
				//(*dKalman)->matriceH[2] * (*dKalman)->matricePpHt[6]

    (*dKalman)->appoggioInv[1] =  (*dKalman)->matriceH[0] * (*dKalman)->matricePpHt[1];
				//(*dKalman)->matriceH[1] * (*dKalman)->matricePpHt[4]
				//(*dKalman)->matriceH[2] * (*dKalman)->matricePpHt[7]
				//(*dKalman)->matriceR[1];

    (*dKalman)->appoggioInv[2] =  (*dKalman)->matriceH[0] * (*dKalman)->matricePpHt[2];
				//(*dKalman)->matriceH[1] * (*dKalman)->matricePpHt[5]
				//(*dKalman)->matriceH[2] * (*dKalman)->matricePpHt[8]
				//(*dKalman)->matriceR[2];

    (*dKalman)->appoggioInv[3] =  (*dKalman)->matriceH[4] * (*dKalman)->matricePpHt[3];
				//(*dKalman)->matriceH[3] * (*dKalman)->matricePpHt[0]
				//(*dKalman)->matriceH[5] * (*dKalman)->matricePpHt[6]
				//(*dKalman)->matriceR[3];

    (*dKalman)->appoggioInv[4] =  (*dKalman)->matriceH[4] * (*dKalman)->matricePpHt[4] +
				  (*dKalman)->matriceR[4];
				//(*dKalman)->matriceH[3] * (*dKalman)->matricePpHt[1]
				//(*dKalman)->matriceH[5] * (*dKalman)->matricePpHt[7]

    (*dKalman)->appoggioInv[5] =  (*dKalman)->matriceH[4] * (*dKalman)->matricePpHt[5];
				//(*dKalman)->matriceH[3] * (*dKalman)->matricePpHt[2]
				//(*dKalman)->matriceH[5] * (*dKalman)->matricePpHt[8]
				//(*dKalman)->matriceR[5];

    (*dKalman)->appoggioInv[6] =  (*dKalman)->matriceH[8] * (*dKalman)->matricePpHt[6];
				//(*dKalman)->matriceH[6] * (*dKalman)->matricePpHt[0]
				//(*dKalman)->matriceH[7] * (*dKalman)->matricePpHt[3]
				//(*dKalman)->matriceR[6];

    (*dKalman)->appoggioInv[7] =  (*dKalman)->matriceH[8] * (*dKalman)->matricePpHt[7];
				//(*dKalman)->matriceH[6] * (*dKalman)->matricePpHt[1]
				//(*dKalman)->matriceH[7] * (*dKalman)->matricePpHt[4]
				//(*dKalman)->matriceR[7];

    (*dKalman)->appoggioInv[8] =  (*dKalman)->matriceH[8] * (*dKalman)->matricePpHt[8] +
				  (*dKalman)->matriceR[8];
				//(*dKalman)->matriceH[6] * (*dKalman)->matricePpHt[2]
				//(*dKalman)->matriceH[7] * (*dKalman)->matricePpHt[5]


    //calcolo determinante matrice da invertire
    //       [a b c]
    //se B = [d e f] det(B) = a*e*i+b*f*g+c*d*h - [c*e*g+a*f*h+b*d*i]
    //       [g h i]
    (*dKalman)->detInv = (*dKalman)->appoggioInv[0] * (*dKalman)->appoggioInv[4] * (*dKalman)->appoggioInv[8] +
			(*dKalman)->appoggioInv[1] * (*dKalman)->appoggioInv[5] * (*dKalman)->appoggioInv[6] +
			(*dKalman)->appoggioInv[2] * (*dKalman)->appoggioInv[3] * (*dKalman)->appoggioInv[7] -
			(*dKalman)->appoggioInv[2] * (*dKalman)->appoggioInv[4] * (*dKalman)->appoggioInv[6] -
			(*dKalman)->appoggioInv[0] * (*dKalman)->appoggioInv[5] * (*dKalman)->appoggioInv[7] -
			(*dKalman)->appoggioInv[1] * (*dKalman)->appoggioInv[3] * (*dKalman)->appoggioInv[8];


    //calcolo matrice inversa = (H*Pp*H'+R)^(-1)
    //                               [ |b22 b23|  |b12 b13|  |b12 b13| ]
    //                               [+|b32 b33| -|b32 b33| +|b22 b23| ]
    //       [b11 b12 b13]           [                                 ]
    //se B = [b21 b22 b23]  B^(-1) = [ |b21 b23|  |b11 b13|  |b11 b13| ]
    //       [b31 b32 b33]           [-|b31 b33| +|b31 b33| -|b21 b23| ]
    //                               [                                 ]
    //dove |Bij Bkl| = det[Bij Bkl]  [ |b21 b22|  |b11 b12|  |b11 b12| ]
    //     |Bmn Bop|      [Bmn Bop]  [+|b31 b32| -|b31 b32| +|b21 b22| ]
    //
    (*dKalman)->detInv = 1/(*dKalman)->detInv;

    (*dKalman)->inversa[0] =                 (*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[4] * (*dKalman)->appoggioInv[8] -
			      (*dKalman)->appoggioInv[5] * (*dKalman)->appoggioInv[7] );

    (*dKalman)->inversa[1] =              -1*(*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[1] * (*dKalman)->appoggioInv[8] -
			      (*dKalman)->appoggioInv[2] * (*dKalman)->appoggioInv[7] );

    (*dKalman)->inversa[2] =                 (*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[1] * (*dKalman)->appoggioInv[5] -
			      (*dKalman)->appoggioInv[2] * (*dKalman)->appoggioInv[4] );

    (*dKalman)->inversa[3] =               -1*(*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[3] * (*dKalman)->appoggioInv[8] -
			      (*dKalman)->appoggioInv[5] * (*dKalman)->appoggioInv[6] );

    (*dKalman)->inversa[4] =                  (*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[0] * (*dKalman)->appoggioInv[8] -
			      (*dKalman)->appoggioInv[2] * (*dKalman)->appoggioInv[6] );

    (*dKalman)->inversa[5] =               -1*(*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[0] * (*dKalman)->appoggioInv[5] -
			      (*dKalman)->appoggioInv[2] * (*dKalman)->appoggioInv[3] );

    (*dKalman)->inversa[6] =                  (*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[3] * (*dKalman)->appoggioInv[7] -
			      (*dKalman)->appoggioInv[4] * (*dKalman)->appoggioInv[6] );

    (*dKalman)->inversa[7] =               -1*(*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[0] * (*dKalman)->appoggioInv[7] -
			      (*dKalman)->appoggioInv[1] * (*dKalman)->appoggioInv[6] );

    (*dKalman)->inversa[8] =                  (*dKalman)->detInv *
			    ( (*dKalman)->appoggioInv[0] * (*dKalman)->appoggioInv[4] -
			      (*dKalman)->appoggioInv[1] * (*dKalman)->appoggioInv[3] );


    //calcolo della matrice di guadagno di kalman, matriceK = Pp*H'*(H*Pp*H'+R)^(-1)
    (*dKalman)->matriceK[0] =  (*dKalman)->matricePpHt[0] * (*dKalman)->inversa[0] +
			      (*dKalman)->matricePpHt[1] * (*dKalman)->inversa[3] +
			      (*dKalman)->matricePpHt[2] * (*dKalman)->inversa[6];

    (*dKalman)->matriceK[1] =  (*dKalman)->matricePpHt[0] * (*dKalman)->inversa[1] +
			      (*dKalman)->matricePpHt[1] * (*dKalman)->inversa[4] +
			      (*dKalman)->matricePpHt[2] * (*dKalman)->inversa[7];

    (*dKalman)->matriceK[2] =  (*dKalman)->matricePpHt[0] * (*dKalman)->inversa[2] +
			      (*dKalman)->matricePpHt[1] * (*dKalman)->inversa[5] +
			      (*dKalman)->matricePpHt[2] * (*dKalman)->inversa[8];

    (*dKalman)->matriceK[3] =  (*dKalman)->matricePpHt[3] * (*dKalman)->inversa[0] +
			      (*dKalman)->matricePpHt[4] * (*dKalman)->inversa[3] +
			      (*dKalman)->matricePpHt[5] * (*dKalman)->inversa[6];

    (*dKalman)->matriceK[4] =  (*dKalman)->matricePpHt[3] * (*dKalman)->inversa[1] +
			      (*dKalman)->matricePpHt[4] * (*dKalman)->inversa[4] +
			      (*dKalman)->matricePpHt[5] * (*dKalman)->inversa[7];

    (*dKalman)->matriceK[5] =  (*dKalman)->matricePpHt[3] * (*dKalman)->inversa[2] +
			      (*dKalman)->matricePpHt[4] * (*dKalman)->inversa[5] +
			      (*dKalman)->matricePpHt[5] * (*dKalman)->inversa[8];

    (*dKalman)->matriceK[6] =  (*dKalman)->matricePpHt[6] * (*dKalman)->inversa[0] +
			      (*dKalman)->matricePpHt[7] * (*dKalman)->inversa[3] +
			      (*dKalman)->matricePpHt[8] * (*dKalman)->inversa[6];

    (*dKalman)->matriceK[7] =  (*dKalman)->matricePpHt[6] * (*dKalman)->inversa[1] +
			      (*dKalman)->matricePpHt[7] * (*dKalman)->inversa[4] +
			      (*dKalman)->matricePpHt[8] * (*dKalman)->inversa[7];

    (*dKalman)->matriceK[8] =  (*dKalman)->matricePpHt[6] * (*dKalman)->inversa[2] +
			      (*dKalman)->matricePpHt[7] * (*dKalman)->inversa[5] +
			      (*dKalman)->matricePpHt[8] * (*dKalman)->inversa[8];


    //calcolo della matrice K*H, KH
    //le moltiplicazioni con valori nulli non le faccio
    //elementi nulli in H in posizione 1,2,3,5,6,7
    (*dKalman)->matriceKH[0] =   (*dKalman)->matriceK[0] * (*dKalman)->matriceH[0];
			      //(*dKalman)->matriceK[1] * (*dKalman)->matriceH[3]
			      //(*dKalman)->matriceK[2] * (*dKalman)->matriceH[6];

    (*dKalman)->matriceKH[1] =   (*dKalman)->matriceK[1] * (*dKalman)->matriceH[4];
			      //(*dKalman)->matriceK[0] * (*dKalman)->matriceH[1]
			      //(*dKalman)->matriceK[2] * (*dKalman)->matriceH[7];

    (*dKalman)->matriceKH[2] =   (*dKalman)->matriceK[2] * (*dKalman)->matriceH[8];
			      //(*dKalman)->matriceK[0] * (*dKalman)->matriceH[2]
			      //(*dKalman)->matriceK[1] * (*dKalman)->matriceH[5]

    (*dKalman)->matriceKH[3] =   (*dKalman)->matriceK[3] * (*dKalman)->matriceH[0];
			      //(*dKalman)->matriceK[4] * (*dKalman)->matriceH[3]
			      //(*dKalman)->matriceK[5] * (*dKalman)->matriceH[6];

    (*dKalman)->matriceKH[4] =   (*dKalman)->matriceK[4] * (*dKalman)->matriceH[4];
			      //(*dKalman)->matriceK[3] * (*dKalman)->matriceH[1]
			      //(*dKalman)->matriceK[5] * (*dKalman)->matriceH[7];

    (*dKalman)->matriceKH[5] =   (*dKalman)->matriceK[5] * (*dKalman)->matriceH[8];
			      //(*dKalman)->matriceK[3] * (*dKalman)->matriceH[2]
			      //(*dKalman)->matriceK[4] * (*dKalman)->matriceH[5]

    (*dKalman)->matriceKH[6] =   (*dKalman)->matriceK[6] * (*dKalman)->matriceH[0];
			      //(*dKalman)->matriceK[7] * (*dKalman)->matriceH[3]
			      //(*dKalman)->matriceK[8] * (*dKalman)->matriceH[6];

    (*dKalman)->matriceKH[7] =   (*dKalman)->matriceK[7] * (*dKalman)->matriceH[4];
			      //(*dKalman)->matriceK[6] * (*dKalman)->matriceH[1]
			      //(*dKalman)->matriceK[8] * (*dKalman)->matriceH[7];

    (*dKalman)->matriceKH[8] =   (*dKalman)->matriceK[8] * (*dKalman)->matriceH[8];
			      //(*dKalman)->matriceK[6] * (*dKalman)->matriceH[2]
			      //(*dKalman)->matriceK[7] * (*dKalman)->matriceH[5]


    //correzione della matrice P, matrice di covarianza dell'errore
    (*dKalman)->matriceP[0] =              (*dKalman)->matricePp[0] -
			      ( (*dKalman)->matriceKH[0] * (*dKalman)->matricePp[0] +
				(*dKalman)->matriceKH[1] * (*dKalman)->matricePp[3] +
				(*dKalman)->matriceKH[2] * (*dKalman)->matricePp[6] );

    (*dKalman)->matriceP[1] =              (*dKalman)->matricePp[1] -
			      ( (*dKalman)->matriceKH[0] * (*dKalman)->matricePp[1] +
				(*dKalman)->matriceKH[1] * (*dKalman)->matricePp[4] +
				(*dKalman)->matriceKH[2] * (*dKalman)->matricePp[7] );

    (*dKalman)->matriceP[2] =              (*dKalman)->matricePp[2] -
			      ( (*dKalman)->matriceKH[0] * (*dKalman)->matricePp[2] +
				(*dKalman)->matriceKH[1] * (*dKalman)->matricePp[5] +
				(*dKalman)->matriceKH[2] * (*dKalman)->matricePp[8] );

    (*dKalman)->matriceP[3] =              (*dKalman)->matricePp[3] -
			      ( (*dKalman)->matriceKH[3] * (*dKalman)->matricePp[0] +
				(*dKalman)->matriceKH[4] * (*dKalman)->matricePp[3] +
				(*dKalman)->matriceKH[5] * (*dKalman)->matricePp[6] );

    (*dKalman)->matriceP[4] =              (*dKalman)->matricePp[4] -
			      ( (*dKalman)->matriceKH[3] * (*dKalman)->matricePp[1] +
				(*dKalman)->matriceKH[4] * (*dKalman)->matricePp[4] +
				(*dKalman)->matriceKH[5] * (*dKalman)->matricePp[7] );

    (*dKalman)->matriceP[5] =              (*dKalman)->matricePp[5] -
			      ( (*dKalman)->matriceKH[3] * (*dKalman)->matricePp[2] +
				(*dKalman)->matriceKH[4] * (*dKalman)->matricePp[5] +
				(*dKalman)->matriceKH[5] * (*dKalman)->matricePp[8] );

    (*dKalman)->matriceP[6] =              (*dKalman)->matricePp[6] -
			      ( (*dKalman)->matriceKH[6] * (*dKalman)->matricePp[0] +
				(*dKalman)->matriceKH[7] * (*dKalman)->matricePp[3] +
				(*dKalman)->matriceKH[8] * (*dKalman)->matricePp[6] );

    (*dKalman)->matriceP[7] =              (*dKalman)->matricePp[7] -
			      ( (*dKalman)->matriceKH[6] * (*dKalman)->matricePp[1] +
				(*dKalman)->matriceKH[7] * (*dKalman)->matricePp[4] +
				(*dKalman)->matriceKH[8] * (*dKalman)->matricePp[7] );

    (*dKalman)->matriceP[8] =              (*dKalman)->matricePp[8] -
			      ( (*dKalman)->matriceKH[6] * (*dKalman)->matricePp[2] +
				(*dKalman)->matriceKH[7] * (*dKalman)->matricePp[5] +
				(*dKalman)->matriceKH[8] * (*dKalman)->matricePp[8] );


    //calcolo vettoreDiff, vettore dato da (Y'-xkp) di dimensione 3x1
    (*dKalman)->vettoreDiff[0] = misure[0] - (*dKalman)->xkp[0];
    (*dKalman)->vettoreDiff[1] = misure[1] - (*dKalman)->xkp[1];

    //riporto tra -pi e +pi
    float vettDiffTemp = misure[2] - (*dKalman)->xkp[2];
    if (vettDiffTemp > M_PI) {
	vettDiffTemp -= (2 * M_PI);
    }
    if (vettDiffTemp<-M_PI) {
	vettDiffTemp += (2 * M_PI);
    }
    (*dKalman)->vettoreDiff[2] = vettDiffTemp;


    //correzione dello stato, calcolo vettore xk
    (*dKalman)->xk[0] =                  (*dKalman)->xkp[0] +
			( (*dKalman)->matriceK[0] * (*dKalman)->vettoreDiff[0] +
			  (*dKalman)->matriceK[1] * (*dKalman)->vettoreDiff[1] +
			  (*dKalman)->matriceK[2] * (*dKalman)->vettoreDiff[2] );

    (*dKalman)->xk[1] =                  (*dKalman)->xkp[1] +
			( (*dKalman)->matriceK[3] * (*dKalman)->vettoreDiff[0] +
			  (*dKalman)->matriceK[4] * (*dKalman)->vettoreDiff[1] +
			  (*dKalman)->matriceK[5] * (*dKalman)->vettoreDiff[2] );

    //riporto tra -pi e +pi
    float xkTemp =  (*dKalman)->xkp[2] +
			( (*dKalman)->matriceK[6] * (*dKalman)->vettoreDiff[0] +
			  (*dKalman)->matriceK[7] * (*dKalman)->vettoreDiff[1] +
			  (*dKalman)->matriceK[8] * (*dKalman)->vettoreDiff[2] );

    if (xkTemp > M_PI) {
	xkTemp -= (2 * M_PI);
    }
    if (xkTemp<-M_PI) {
	xkTemp += (2 * M_PI);
    }
    (*dKalman)->xk[2] = xkTemp;


    return 0;
}



/* ----------------------- Funzioni ausiliarie ---------------------- */

//=====================================================================
//                          Fase iniziale
//=====================================================================

//Inizializza tutti i dati necessari per Kalman
int inizializzaDatiKalman(datiKalman **dKalman) {

    // alloco la struttura datiKalman
    *dKalman = (datiKalman *)malloc( sizeof( datiKalman ) );

	if ( *dKalman == NULL)
	{
		printf("inizializzaDati: Non è possibile creare datiKalman (out of memory?)...\n");
		return -1;
	}

    FILE        *fmatriceQ;
    FILE        *fmatriceR;
    FILE        *fmatriceP;
    FILE        *fvettoreXk;
    FILE        *ftabellaSeno;
    int riga;


    // Q matrice di covarianza del rumore di processo (fissa)
    // apertura file matriceQ.txt
    fmatriceQ=fopen(MATRICEQ,"r");
    if(fmatriceQ==NULL) {
	printf("errore apertura file matriceQ.txt\n");
	return -1;
    }
    //Inizializzo Q
    (*dKalman)->matriceQ = (float*)malloc(sizeof(float)* 3 * 3);
    // riempimento matriceQ
    riga=0;
    while(fscanf(fmatriceQ,"%f %f %f\n",&(*dKalman)->matriceQ[(riga*3)], &(*dKalman)->matriceQ[(riga*3)+1], &(*dKalman)->matriceQ[(riga*3)+2])!=EOF) {
	riga++;
    }
    // chiusura file
    fclose(fmatriceQ);


    // R matrice di covarianza del rumore di misura (fissa)
    // apertura file matriceR.txt
    fmatriceR=fopen(MATRICER,"r");
    if(fmatriceR==NULL) {
	printf("errore apertura file matriceR.txt\n");
	return -1;
    }
	//Inizializzo R
	(*dKalman)->matriceR = (float*)malloc(sizeof(float)* 3 * 3);
    // riempimento matriceR
    riga=0;
    while(fscanf(fmatriceR,"%f %f %f\n",&(*dKalman)->matriceR[(riga*3)], &(*dKalman)->matriceR[(riga*3)+1], &(*dKalman)->matriceR[(riga*3)+2])!=EOF) {
	riga++;
    }
    // chiusura file
    fclose(fmatriceR);


    // P matrice di covarianza dell'errore (evolve)
    // apertura file matriceP.txt
    fmatriceP=fopen(MATRICEP,"r");
    if(fmatriceP==NULL) {
	printf("errore apertura file matriceP.txt\n");
	return -1;
    }
	//Inizializzo P
	(*dKalman)->matriceP = (float*)malloc(sizeof(float)* 3 * 3);
    // riempimento matriceP
    riga=0;
    while(fscanf(fmatriceP,"%f %f %f\n",&(*dKalman)->matriceP[(riga*3)], &(*dKalman)->matriceP[(riga*3)+1], &(*dKalman)->matriceP[(riga*3)+2])!=EOF) {
	riga++;
    }
    // chiusura file
    fclose(fmatriceP);


    // Pp matrice di predizione della covarianza dell'errore
    //Alloco matricePp
    (*dKalman)->matricePp = (float*)malloc(sizeof(float)* 3 * 3);


    // A matrice Jacobiano delle derivate parziali di f rispetto allo stato
    //Alloco matriceA
    (*dKalman)->matriceA = (float*)malloc(sizeof(float)* 3 * 3);
    //riempo A come identità
    (*dKalman)->matriceA[0] = 1.0;
    (*dKalman)->matriceA[1] = 0.0;
    (*dKalman)->matriceA[2] = 0.0;
    (*dKalman)->matriceA[3] = 0.0;
    (*dKalman)->matriceA[4] = 1.0;
    (*dKalman)->matriceA[5] = 0.0;
    (*dKalman)->matriceA[6] = 0.0;
    (*dKalman)->matriceA[7] = 0.0;
    (*dKalman)->matriceA[8] = 1.0;


    // calcolo matrice A * P
    //Alloco matriceAP
    (*dKalman)->matriceAP = (float*)malloc(sizeof(float)* 3 * 3);


    // H matrice Jacobiano delle derivate parziali di h rispetto allo stato
    // nel nostro caso H è fissa ed è pari a
    // [1][0][0]
    // [0][1][0]
    // [0][0][1]
    //Alloco H
    (*dKalman)->matriceH = (float*)malloc(sizeof(float)* 3 * 3);
    //riempo H come identità
    (*dKalman)->matriceH[0] = 1.0;
    (*dKalman)->matriceH[1] = 0.0;
    (*dKalman)->matriceH[2] = 0.0;
    (*dKalman)->matriceH[3] = 0.0;
    (*dKalman)->matriceH[4] = 1.0;
    (*dKalman)->matriceH[5] = 0.0;
    (*dKalman)->matriceH[6] = 0.0;
    (*dKalman)->matriceH[7] = 0.0;
    (*dKalman)->matriceH[8] = 1.0;


    // matrice Pp*H'
    //Alloco matricePpHt
    (*dKalman)->matricePpHt = (float*)malloc(sizeof(float)* 3 * 3);


    // matrice appoggioInv = H*Pp*H'+R
    //Alloco appoggioInv
    (*dKalman)->appoggioInv = (float*)malloc(sizeof(float)* 3 * 3);


    // matrice inversa = (H*Pp*H'+R)^(-1)
    //Alloco inversa
    (*dKalman)->inversa = (float*)malloc(sizeof(float)* 3 * 3);


    // xk vettore contenente lo stato attuale del filtro di kalman
    fvettoreXk=fopen(VETTOREXK,"r");
    if(fvettoreXk==NULL) {
	printf("errore apertura file vettoreXk.txt\n");
	return -1;
    }
    //Alloco xk
	(*dKalman)->xk = (float*)malloc(sizeof(float)*3);
	// riempimento xk
    fscanf(fvettoreXk,"%f %f %f\n",&(*dKalman)->xk[0], &(*dKalman)->xk[1], &(*dKalman)->xk[2]);
    // chiusura file
    fclose(fvettoreXk);


    // xkp vettore contenente lo stato predetto del filtro di kalman
    //Alloco xkp
	(*dKalman)->xkp = (float*)malloc(sizeof(float)*3);


    // K matrice di guadagno di kalman
    //Alloco matriceK
    (*dKalman)->matriceK = (float*)malloc(sizeof(float)* 3 * 3);


    //matrice K * H
    //Alloco matriceKH
    (*dKalman)->matriceKH = (float*)malloc(sizeof(float)* 3 * 3);


    //vettoreDiff vettore dato da (Y'-xkp) di dimensione 3x1
    //Alloco vettoreDiff
    (*dKalman)->vettoreDiff = (float*)malloc(sizeof(float)*3);


    //Inizializzo tabellaSeno
    // apertura file tabellaSeno.txt
    ftabellaSeno=fopen(TABELLASENO,"r");
    if(ftabellaSeno==NULL) {
	printf("errore apertura file tabellaSeno.txt\n");
	return -1;
    }
	//Inizializzo tabellaSeno
	(*dKalman)->tabellaSeno = (float*)malloc(sizeof(float)* 900);
    // riempimento tabellaSeno
    riga=0;
    while(fscanf(ftabellaSeno,"%f\n",&(*dKalman)->tabellaSeno[riga])!=EOF) {
	riga++;
    }
    // chiusura file
    fclose(ftabellaSeno);



    return 0;
}

void distruggiDatiKalman( datiKalman *dKalman ) {

	free( dKalman->matriceQ );
	free( dKalman->matriceR );
	free( dKalman->matriceP );
	free( dKalman->matriceA );
	free( dKalman->matriceAP );
	free( dKalman->matricePp );
	free( dKalman->matriceH );
	free( dKalman->matricePpHt );
	free( dKalman->appoggioInv );
	free( dKalman->inversa );
	free( dKalman->xk );
	free( dKalman->xkp );
	free( dKalman->matriceK );
	free( dKalman->matriceKH );
	free( dKalman->vettoreDiff );
	free( dKalman->tabellaSeno );
	free( dKalman );
}

//=====================================================================
//                       Operazioni su matrici
//=====================================================================
//funzione per stampare a video la matrice.
void stampaMatrice(char testo,float *matricedastampare, int righe, int colonne) {

	//iteratori
	int i,j;

    if (righe>1) {
	printf("\nmatrice %c \n\n",testo);
    } else {
	printf("\nvettore %c \n\n",testo);
    }

	//iterazioni di stampa su video
    j=0;
	for(i=0;i<(righe*colonne);i++) {

	    printf(" %f",matricedastampare[i]);
	    if(j==colonne-1) {
		printf("\n");
		j=0;
	    } else j++;

	}

}


//=====================================================================
//                        Calcoli matematici
//=====================================================================

/*
funzioni per calcolare seno e coseno mediante una tabella.

Caratteristiche:
		-discretizzazione fino ad un 0.2 di angolo (0.2°)
		-range da 0° a 359.8°
		-tabella di 1800 valori
		-arrotondamento del valore decimale diverso da 0.2 dell'angolo

*/




//Funzione per calcolare sen(angolo) tramite una tabella (seno discretizzato con granularità di 0.2°)
float calcolaSeno(datiKalman **dKalman, float angolo) {


    //individuazione della posizione e resto
    float posizionef = 0;
    //variabile in cui memorizzo l'output
    float output;
    //arrotondamento per difetto
    int posizione;

    if (angolo>0.0) {
	posizionef = angolo/0.003490;
	posizione = (int)posizionef;
	output = (*dKalman)->tabellaSeno[posizione];
    } else if (angolo<0.0) {
	posizionef = (angolo*-1)/0.003490;
	posizione = (int)posizionef;
	output = ((*dKalman)->tabellaSeno[posizione])*-1;
    } else output = 0.0;

    return output;

}

//Funzione per calcolare cos(angolo) tramite una tabella (seno discretizzato con granularità di 0.2°)
float calcolaCoseno(datiKalman **dKalman, float angolo) {


    //individuazione della posizione e resto
    float posizionef;
    //variabile in cui memorizzo l'output
    float output;
    //arrotondamento per difetto
    int posizione;
    int segno = 1;

    posizionef = 0;
    if (angolo>0.0) {
	posizionef = angolo/0.003490;
    } else if (angolo<0.0) {
	posizionef = (angolo*-1)/0.003490;
    }
    posizione = (int)posizionef+450;
    if (posizione>899) {
	posizione = posizione-900;
	segno = -1;
    }
    output = segno*(*dKalman)->tabellaSeno[posizione];

    return output;

}


