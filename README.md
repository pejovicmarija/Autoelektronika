# **Upravljanje_svjetlima_u_automobilu**

## **Opis projekta**
Napraviti softver za simulaciju upravljanja svetlima u automobilu. Postoje dva rezima rada: MANUELNI i AUTOMATSKI
Kod Manuelnog sami regulisemo farove preko prekidaca
Kod Automatskog pratimo vrijednosti dobijene sa senzora i na osnovu usrednjene vrijednosti (oisljednjih 10 odbiraka) kontrolisemo rad farova
Pored ovoga imamo displej koji prikazuje rezim rada, trenutnu vrijednost osvjetljenja i minimalnu ili maksimalnu vrednost osvjetljenja u zavisnosti od pritisnutog tastera

## **Koriscene periferije**
- _AvdUniCom_ koji ima 3 kanala. 0 predstavlja trenutno stanje senzora kolicine spoljasnjeg osvjetljenja, 1 trenutno stanje otvorenosti vrata a 2 sluzi za ispis relevantnih informacija (rezim rada, ukljuceno/iskljuceno, trenutna osvjetljenost, da li je neki far ukljucen). Na kanalu 2 upisujemo koji rezim zelimo da izaberemo

- _Seg7Mux_ kao sto je prethodno spomenuto prikazuje rezime rada (0-AUTOMATSKI i 1-MANUELNO), trenutnu osvjetljenost i minimalnu ili maksimalnu vrijednost osvjetljenosti

- _LED bars plus_ prvi stup koji se sastoji od 8 LED dioda predstavlja je ulazni i simulira prekidace, a drugi je izlazni i ako je donja LED ukljucena, ukljucena su dnevna svetla. Sledeca LED se koristi za kratka svetla, potom duga, pa levi i desni zmigavac. Ako rade dnevna svetla, ukljuciti gornju LED izlaznog stupca. Druga od gore simulira ukljucena kratka svetla, potom idu duga. Cetvrta i peta LED od gore simuliraju rad zmigavaca koji treba da blinkaju periodom od 500ms ukoliko su ukljuceni.

## **Pokretanje programa**
- Preuzeti ceo repozitorijum sa Github sajta. Neophodan je instaliran Visual Studio 2019 kao okruzenje u kom se radi i PC lint kao program za staticku analizu koda.
- Otvoriti Visual Studio, kliknuti na komandu Open a local folder i otvoriti preuzeti repozitorijum. Zatim, sa desne strane kliknuti na FreeRTOS_simulator_final.sln, a onda na foldere Starter i Source Files i na fajl main.application kako bi se prikazao kod u radnom prostoru.
- U gornjem srednjem delu ispod Analyze podesiti x86 i pokrenuti softver pomocu opcije Local Windows Debugger koja se nalazi pored.

## **Pokretanje periferija**
- Preko Command Prompt-a uci u folder Periferije. 
- U folderu AdvUniCom pokrenuti tri puta aplikaciju AdvUniCom i kao argument svaki put proslediti po jedan broj (0,1,2) da bi se otvorio odgovarajuci kanal. Za nulti kanal ne treba da se prosledi broj
- Zatim u folderu LEDbars pokrenuti aplikaciju LED_bars_plus i kao argument proslediti rB
- Zatim u folderu 7SegMux pokrenuti aplikaciju Seg7_Mux i kao argument proslediti broj 5
- Sada ste spremni za testiranje softvera

## **Upustvo za testiranje softvera**
---U COM0 umesto T1 uneti XYZ i oznaciti Auto 1, a zatim umesto R1 uneti simulirane podatke sa senzora koji pocinju sa \fe, a zavrsavaju sa \0d. Izmedju pisemo koja je ocitana vrednost osvjetljenosti. 
- Kod COM1 se jedino razlikuje polje T1 gde treba da se unese ABC, ostalo je isto kao i kod COM0
- Na COM2 softver automatski pretpostavi da se nalazimo u manuelnom rezimu, ako zelimo da se prebacimo u automatski rezim pisemo 

---Ako je trenutni rezim rada MANUELNO, na osnovu pritisnutog prekidaca ulaznog stubca svijetle nam odredjene diode koje simuliraju redom: dnevna, kratka i duga svjetla, lijevi i desni zmigavac
---Ako je trenutni rezim rada AUTOMATSKI, svjetla se kontrolisu prema senzoru osvjetljenja. Ako je trenutna vrijednost osvjetljenja (COM0) veca od 500 gase se kratka svjetla nakon 5s. Svjetla u kabini se gase ako se na COM1 posalje 1 (otvorena vrata), a pale se ako se na COM1 posalje 0 (zatvorena vrata)
---Na 7Seg Displjeju se prikazuje trenutna vrijednost koju saljemo preko CH0. Ako je pritisnut prvi ulazni taster na Led Baru, na 7Seg Displeju se ispisuje minimalna vrijednost, a ako je pritisnut drugi taster ispisuje se maksimalna vrijednost. 
