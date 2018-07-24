package maze51;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;
import javax.swing.*;
import javax.swing.event.*;

/**
 * Το πρόγραμμα λύνει και οπτικοποιεί το πρόβλημα του σχεδιασμού κίνησης ρομπότ
 * (robot motion planning) υλοποιώντας μια παραλλαγή των αλγόριθμων
 * DFS, BFS και A*, όπως αυτοί περιγράφονται στο βιβλίο
 * "Τεχνητή Νοημοσύνη και Έμπειρα Συστήματα" της Ε. Κεραυνού, ΠΑΤΡΑ 2000
 * καθώς και τον αλγόριθμο της άπληστης αναζήτησης, σαν ειδική περίπτωση του Α*.
 * 
 * Γίνεται ολοφάνερη η υπεροχή του Αλγορίθμου Α* απέναντι στους άλλους δύο.
 * 
 * Ο χρήστης μπορεί μεταβάλει το πλήθος των κελιών του πλέγματος, δηλώνοντας
 * τον επιθυμητό αριθμό γραμμών και στηλών.
 * 
 * Ο χρήστης μπορεί να προσθέσει όσα εμπόδια θέλει, όπως θα σχεδίαζε ελεύθερες
 * καμπύλες με ένα σχεδιαστικό πρόγραμμα.
 * 
 * Αφαίρεση μεμονωμένων εμποδίων γίνεται κάνοντας κλικ επάνω τους.
 * 
 * Η θέση του ρομπότ ή/και του στόχου μπορεί να αλλάξει με σύρσιμο με το ποντίκι.
 * 
 * Μεταπήδηση από την αναζήτηση "Βήμα-Βήμα" στην αναζήτηση με "Κίνηση" και αντίστροφα
 * γίνεται πιέζοντας το αντίστοιχο κουμπί, ακόμη και όταν η αναζήτηση είναι σε εξέλιξη.
 * 
 * Η ταχύτητα μιας αναζήτησης με κίνηση μπορεί να μεταβληθεί, ακόμη και αν η
 * αναζήτηση είναι σε εξέλιξη, αρκεί να τοποθετηθεί ο  slider "Ταχύτητα" στην νέα
 * επιθυμητή θέση και στη συνέχεια πιεστεί το κουμπί "Κίνηση".
 * 
 * Η εφαρμογή θεωρεί ότι το ίδιο το ρομπότ έχει κάποιον όγκο. Συνεπώς δεν μπορεί
 * να κινηθεί διαγώνια προς ελεύθερο κελί ανάμεσα από δύο εμπόδια που εφάπτονται
 * σε μία κορυφή τους.
 *
 * Δεν είναι δυνατή η αλλαγή των θέσεων εμποδίων, ρομπότ και στόχου όπως και του
 * είδους του αλγόριθμου, ενόσω η αναζήτηση είναι σε εξέλιξη.
 * 
 * Συνιστάται η αποφυγή σχεδίασης βελών προς προκατόχους σε πλέγματα μεγάλης διάστασης.
 */

public class Maze51 
{
    /** Η κύρια φόρμα του προγράμματος
     * 
     */
    public static JFrame mazeFrame;  
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        int width  = 693;
        int height = 545;
        mazeFrame = new JFrame("Maze 5.0");
        mazeFrame.setContentPane(new MazePanel(width,height));
        mazeFrame.pack();
        mazeFrame.setResizable(false);

        // τοποθετούμε τη φόρμα στο κέντρο της οθόνης
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        double screenWidth = screenSize.getWidth();
        double ScreenHeight = screenSize.getHeight();
        int x = ((int)screenWidth-width)/2;
        int y = ((int)ScreenHeight-height)/2;

        mazeFrame.setLocation(x,y);
        mazeFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        mazeFrame.setVisible(true);
    } // end main()
    
    /**
     * Αυτή η κλάση καθορίζει το περιεχόμενο της κύριας φόρμας
     * και περιέχει όλη την λειτουργικότητα του προγράμματος.
     */
    public static class MazePanel extends JPanel 
    {
        
        /**
         **********************************************************
         *          Ένθετες κλάσεις στην MazePanel
         **********************************************************
         */
        
        /**
         * Βοηθητική κλάση που αναπαριστά το κελί του πλέγματος
         */
        private class Cell 
        {
            /** ο αριθμός γραμμής του κελιού (Η γραμμή 0 είναι η πάνω)*/
            int row;   
            /** ο αριθμός στήλης του κελιού (Η στήλη 0 είναι η αριστερή)*/
            int col;   
            /** η τιμή της συνάρτησης g του αλγορίθμου Α* (Το κόστος κίνησης από τον έναν κόμβο στον άλλον)*/
            double g;     
            /** η τιμή της συνάρτησης h του αλγορίθμου Α* (Η απόσταση μέχρι τον προορισμό)*/
            double h;     
            /** η τιμή της συνάρτησης f του αλγορίθμου Α* (g + h)*/
            double f;     
            
            /** κάθε κατάσταση αντιστοιχεί σε κάποιο cell
                        και κάθε κατάσταση έχει μια προκάτοχο η οποία
                        αποθηκεύεται σε αυτή τη μεταβλητή*/           
            Cell prev; 
            
            /** Cell Constructor*/
            public Cell(int row, int col) 
            {
               this.row = row;
               this.col = col;
            }
        } // τέλος βοηθητικής κλασης Cell
      
        /**
         * Βοηθητική κλάση(μέθοδος) που καθορίζει ότι τα κελιά θα ταξινομούνται
         * με βάση το πεδίο τους f
         */
        private class CellComparatorByF implements Comparator<Cell>
        {
            @Override
            public int compare(Cell cell1, Cell cell2)
            {
                return (int)(cell1.f-cell2.f);
            }
        } // τέλος βοηθητικής κλασης CellComparatorByF    
      
        /**
         * Κλάση που χειρίζεται τις κινήσεις του ποντικιού καθώς "ζωγραφίζουμε"
         * εμπόδια ή μετακινούμε το ρομπότ ή/και τον στόχο.
         */
        private class MouseHandler implements MouseListener, MouseMotionListener 
        {
            /**Μεταβλητές : τρέχων γραμμή, τρέχων στήλη, και τρέχων τιμή η τιμή που προκύπτει με το πάτημα του κουμπιου(η τιμή που θα περιέχουν οι συντεταγμένες)*/
            private int cur_row, cur_col, cur_val;
            @Override
            public void mousePressed(MouseEvent evt) //με το πάτημα του κουμπιού 
            {
                //πάρε τις συντεταγμένες με το πάτημα
                int row = (evt.getY() - 10) / squareSize;
                int col = (evt.getX() - 10) / squareSize;
                if (row >= 0 && row < rows && col >= 0 && col < columns) //εαν βρίσκεται μέσα τα όρια του λαβυρίνθου 
                {
                    //εαν το realtime είναι ίσο με το real_time τότε γράψε true αλλιώς δεν βρέθηκε και η αναζήτηση έφτασε στο τέλος
                    boolean real_time = realTime ? true : !found && !searching;
                    if (real_time) 
                    {
                        if (realTime) //ενεργοποιείται μόνο όταν πατήσουμε το κουμπί "Σε πραγματικό χρόνο"
                        {
                            searching = true;
                            fillGrid();
                        }
                        cur_row = row; //η τρέχουσα γραμμή
                        cur_col = col; //η τρέχουσα στήλη
                        cur_val = grid[row][col]; //η τιμή που θα έχουν οι συντεταγμένες
                        //με το πάτημα εαν το κουτι είναι άδειο(ΑΣΠΡΟ) να το κάνεις (ΜΑΥΡΟ)(Με εμπόδια)
                        if (cur_val == EMPTY) 
                        {
                            grid[row][col] = OBST;
                        }
                        //με το πάτημα εαν το κουτι είναι γεμάτο(ΜΑΥΡΟ) να το κάνεις (ΑΣΠΡΟ)(Χωρίς εμπόδια)
                        if (cur_val == OBST)
                        {
                            grid[row][col] = EMPTY;
                        }                        
                    }
                }
                if (realTime) 
                {
                    timer.setDelay(0);
                    timer.start();                   
                } else 
                {
                    repaint();
                }
            }

            @Override
            public void mouseDragged(MouseEvent evt) //όταν σέρνουμε το mouse
            {
                /**πάρε τις συντεταγμένες με το πάτημα που θα κάνεις*/
                int row = (evt.getY() - 10) / squareSize;  
                int col = (evt.getX() - 10) / squareSize;
                if (row >= 0 && row < rows && col >= 0 && col < columns)
                {
                     if (realTime ? true : !found && !searching)
                    {
                        if (realTime) 
                        {
                            searching = true;
                            fillGrid();
                        }
                        //εαν βρίσκεσαι στα όρια του λαβυρίνθου και η τωρινή τιμή είναι είτε η αρχική κατάσταση είτε η τελική
                        if ((row*columns+col != cur_row*columns+cur_col) && (cur_val == ROBOT || cur_val == TARGET))
                        {
                            /** new_val = η τιμη του κελιού (συντεταγμένες) που προκύπτει απο το συρσιμο του mouse*/
                            int new_val = grid[row][col];
                             //εαν η τιμή είναι άδεια->0
                            if (new_val == EMPTY) 
                            {
                                grid[row][col] = cur_val; //όρισε την παλια τιμη στις νεες συντεταγμένες (την τιμή την έχει πάρει από την μέθοδο mousePressed)
                                if (cur_val == ROBOT) {  //εαν η τιμή είναι αυτή του ρομπότ δηλαδή 2...
                                    robotStart.row = row; //το robotstart είναι αντικείμενο τύπου Cell και αποθηκευουμε μια τιμή στην μεταβλητή του row
                                    robotStart.col = col;//το robotstart είναι αντικείμενο τύπου Cell και αποθηκευουμε μια τιμή στην μεταβλητή του col
                                } else 
                                {
                                    targetPos.row = row;
                                    targetPos.col = col;
                                }
                                grid[cur_row][cur_col] = new_val; //βάλε στις παλιες συντεταγμενες την νεα τιμη που προεκυψε
                                cur_row = row;
                                cur_col = col;
                                if (cur_val == ROBOT) 
                                {
                                    robotStart.row = cur_row;
                                    robotStart.col = cur_col;
                                } else 
                                {
                                    targetPos.row = cur_row;
                                    targetPos.col = cur_col;
                                }
                                cur_val = grid[row][col];
                            }
                        } else if (grid[row][col] != ROBOT && grid[row][col] != TARGET) //εαν το κελί δεν είναι ουτε η αρχικη κατάσταση αλά ούτε η τελική 
                        {
                            grid[row][col] = OBST;   //όρισε το κελί να είναι εμπόδιο
                        }                        
                    }
                }
                if (realTime) 
                {
                    timer.setDelay(0);
                    timer.start();                   
                } else 
                {
                    repaint();
                }
            }

            @Override
            public void mouseReleased(MouseEvent evt) { }
            @Override
            public void mouseEntered(MouseEvent evt) { }
            @Override
            public void mouseExited(MouseEvent evt) { }
            @Override
            public void mouseMoved(MouseEvent evt) { }
            @Override
            public void mouseClicked(MouseEvent evt) { }
            
        } // τέλος βοηθητικής κλάσης MouseHandler
        
        /**
         * Όταν ο χρήστης πιέζει ένα κουμπί εκτελεί την αντίστοιχη λειτουργικότητα
         */
        private class ActionHandler implements ActionListener {
            @Override
            public void actionPerformed(ActionEvent evt) {
                String cmd = evt.getActionCommand();
                if (cmd.equals("Καθάρισμα")) {
                    fillGrid();
                    realTime = false;
                    realTimeButton.setEnabled(true);
                    realTimeButton.setForeground(Color.black);
                    stepButton.setEnabled(true);
                    animationButton.setEnabled(true);
                    slider.setEnabled(true);
                    dfs.setEnabled(true);
                    bfs.setEnabled(true);
                    aStar.setEnabled(true);                                  
                    diagonal.setEnabled(true);
                    drawArrows.setEnabled(true);
                    drawNumbers.setEnabled(true);
                } else if (cmd.equals("Σε πραγματικό χρόνο") && !realTime) {
                    realTime = true;  //μόνο εδώ το realtime ειναι true
                    searching = true;
                    realTimeButton.setForeground(Color.red);
                    stepButton.setEnabled(false);
                    animationButton.setEnabled(false);
                    slider.setEnabled(false);
                    dfs.setEnabled(false);
                    bfs.setEnabled(false);
                    aStar.setEnabled(false);                                   
                    diagonal.setEnabled(false);
                    drawArrows.setEnabled(false);
                    drawNumbers.setEnabled(false);
                    timer.setDelay(0);
                    timer.start();                      
                } else if (cmd.equals("Βήμα - Βήμα") && !found && !endOfSearch) {
                    realTime = false;                    
                    searching = true;
                    message.setText(msgSelectStepByStepEtc);
                    realTimeButton.setEnabled(false);
                    dfs.setEnabled(false);
                    bfs.setEnabled(false);
                    aStar.setEnabled(false);                                     
                    diagonal.setEnabled(false);
                    drawArrows.setEnabled(false);
                    drawNumbers.setEnabled(false);
                    timer.stop();
                    // Εδώ αποφασίζουμε αν μπορούμε να συνεχίσουμε την
                    // 'Βήμα-Βήμα' αναζήτηση ή όχι
                    // Για την περίπτωση των αλγόριθμων DFS, BFS, A* 
                    // εδώ έχουμε το 2ο βήμα τους:
                    // 2. Εάν ΑΝΟΙΚΤΕΣ = [], τότε τερμάτισε. Δεν υπάρχει λύση.
                     checkTermination();  //έλεγχος εαν έχει βρεθεί η λύση
                    repaint();
                } else if (cmd.equals("Κίνηση") && !endOfSearch) {
                    realTime = false;                   
                    searching = true;
                    message.setText(msgSelectStepByStepEtc);
                    realTimeButton.setEnabled(false);
                    dfs.setEnabled(false);
                    bfs.setEnabled(false);
                    aStar.setEnabled(false);                                 
                    diagonal.setEnabled(false);
                    drawArrows.setEnabled(false);
                    drawNumbers.setEnabled(false);
                    timer.setDelay(delay);
                    timer.start();
                } else if (cmd.equals("Σχετικά με το Maze")) {
                    AboutBox aboutBox = new AboutBox(mazeFrame,true);
                    aboutBox.setVisible(true);
                }
            }
        } // τέλος βοηθητικής κλάσης ActionHandler
   
        /**
         * Η κλάση που είναι υπεύθυνη για το animation
         */
        private class RepaintAction implements ActionListener {
            @Override
            public void actionPerformed(ActionEvent evt) {
                // Εδώ αποφασίζουμε αν μπορούμε να συνεχίσουμε ή όχι
                // την αναζήτηση με 'Κίνηση'.
                // Για την περίπτωση των αλγόριθμων DFS, BFS, και A* 
                // εδώ έχουμε το 2ο βήμα τους:
                // 2. Εάν ΑΝΟΙΚΤΕΣ = [], τότε τερμάτισε. Δεν υπάρχει λύση.
                checkTermination();
                if (found) {
                    timer.stop();
                }
                if (!realTime) {
                    repaint();
                }
            }
        } // τέλος βοηθητικής κλάσης RepaintAction
      
        /**Έλεγχος Έαν έχει βρεθεί λύση η όχι  */
        public void checkTermination() 
        {
            if (openSet.isEmpty() ) //εαν δεν υπάρχουν ανοιχτες καταστασεις στην λίστα(δηλαδη εαν ειναι κενη) 
            { //δεν έχει βρεθεί η λύση
                endOfSearch = true;  //τερμάτισε την αναζήτηση
                grid[robotStart.row][robotStart.col]=ROBOT;
                message.setText(msgNoSolution);
                stepButton.setEnabled(false);
                animationButton.setEnabled(false);
                slider.setEnabled(false);
                repaint();
            } else //εαν είναι γεμάτη η λίστα 
            { 
                expandNode();
                if (found) //τσέκαρε να δεις αν βρέθηκε η λύση (Εαν βρέθηκε ...)
                {
                    endOfSearch = true; //τερμάτησε την αναζήτηση)
                    plotRoute();        //υπολόγισε την διαδρομή
                    stepButton.setEnabled(false);
                    animationButton.setEnabled(false);
                    slider.setEnabled(false);
                    repaint();
                }
            }
        }

        /**
         * Η κλάση που δημιουργεί το AboutBox
         */
        private class AboutBox extends JDialog{

            public AboutBox(Frame parent, boolean modal){
                super(parent, modal);
                // Το aboutBox τοποθετείται στο κέντρο της οθόνης
                Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
                double screenWidth = screenSize.getWidth();
                double ScreenHeight = screenSize.getHeight();
                int width = 350;
                int height = 190;
                int x = ((int)screenWidth-width)/2;
                int y = ((int)ScreenHeight-height)/2;
                setSize(width,height);
                setLocation(x, y);
         
                setResizable(false);
                setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
 
                JLabel title = new JLabel("Maze", JLabel.CENTER);
                title.setFont(new Font("Helvetica",Font.PLAIN,24));
                title.setForeground(new java.awt.Color(255, 153, 102));
 
                JLabel version = new JLabel("Έκδοση: 5.0", JLabel.CENTER);
                version.setFont(new Font("Helvetica",Font.BOLD,14));
 
                JLabel programmer = new JLabel("Ευάγγελος Νικόλης, ΜΠΠΛ16016", JLabel.CENTER);
                programmer.setFont(new Font("Helvetica",Font.PLAIN,16));
 
                JLabel programmer2 = new JLabel("Ευάγγελος Μαυρόπουλος, ΜΠΠΛ16012", JLabel.CENTER);
                programmer2.setFont(new Font("Helvetica",Font.PLAIN,16));               
 
                JLabel lesson = new JLabel("Τεχνιτή Νοημοσύνη - Έμπειρα Συστήματα", JLabel.CENTER);               
                lesson.setFont(new Font("Helvetica",Font.BOLD,16));          
 
                JLabel dummy = new JLabel("");
 
                add(title);
                add(version);
                add(programmer);
                add(programmer2);
                add(lesson);               
                add(dummy);      
 
                title.      setBounds(5,  0, 330, 30);
                version.    setBounds(5, 30, 330, 20);
                programmer. setBounds(5, 55, 330, 20);
                programmer2.setBounds(5, 80, 330, 20);
                lesson.     setBounds(5,105, 330, 20);       
                dummy.      setBounds(5,155, 330, 20);
            }
 
           
        } // τέλος βοηθητικής κλάσης AboutBox

        /**
         * Δημιουργεί έναν τυχαίο, τέλειο (χωρίς κύκλους) λαβύρινθο         
         */
        private class MyMaze 
        {
            /**Διαστάσεις του Λαβυρίνθου(maze) */
            private int dimensionX, dimensionY;
             /**Διαστάσεις του εξωτερικού πλέγματος(grid) */
            private int gridDimensionX, gridDimensionY;
             /**Εξωτερικό πλέγμα(grid) */
            private char[][] mazeGrid;
             /**Δισδιάστατος(2d) πίνακας από κελιά(Cells) */
            private Cell[][] cells;
             /**Το τυχαίο αντικείμενο */
            private Random random = new Random(); 

             /**Αρχικοποίηση με x και y το ίδιο */
            public MyMaze(int aDimension) {
                // Αρχικοποίηση
                this(aDimension, aDimension);
            }
             /**Constructor */
            public MyMaze(int xDimension, int yDimension) 
            {
                dimensionX = xDimension;
                dimensionY = yDimension;
                gridDimensionX = xDimension * 2 + 1;
                gridDimensionY = yDimension * 2 + 1;
                mazeGrid = new char[gridDimensionX][gridDimensionY];
                init();
                generateMaze();
            }

            private void init() 
            {
                 /**Δημιουργία κελιών */
                cells = new Cell[dimensionX][dimensionY];
                for (int x = 0; x < dimensionX; x++) {
                    for (int y = 0; y < dimensionY; y++) {
                         /**Δημιουργία κελιών(Δες Cell constructor)  */
                        cells[x][y] = new Cell(x, y, false); 
                    }
                }
            }

            /** Εσωτερική κλάση η οποία εκπροσωπεί ένα κελί */
            private class Cell 
            {
                /**Συντεταγμένες */
                int x, y; 
                /** Λίστα τύπου Cell στη οποία αποθηκεύονται τα δεδομένα κάθε κελιού*/
                ArrayList<Cell> neighbors = new ArrayList<>();
                /** αδιαπέραστο κελί*/
                boolean wall = true;
                /** Εαν είναι αλήθεια, μπορεί ακόμα να χρησιμοποιηθεί στην παραγωγή*/
                boolean open = true;
                /** construct Cell στις θέσεις x, y */
                Cell(int x, int y) 
                {
                    this(x, y, true);
                }
                /** construct Cell στις θέσεις x, y και με αν υπάρχει η όχι τοίχος(isWall)*/
                Cell(int x, int y, boolean isWall) 
                {
                    this.x = x;
                    this.y = y;
                    this.wall = isWall;
                }
                /**Πρόσθεσε έναν γείτονα σε αυτό το κελί, και πρόσθεσε αυτό το κελί σαν γείτονα σε ένα άλλο*/
                void addNeighbor(Cell other) {
                    if (!this.neighbors.contains(other)) { // αποφυγή διπλότυπων στη λίστα μας (εαν δεν περιέχει την τιμή) 
                        this.neighbors.add(other);          //βάλτο μες τη λίστα
                    }
                    if (!other.neighbors.contains(this)) { // αποφυγή διπλότυπων
                        other.neighbors.add(this);
                    }
                }
                /** Χρησιμοποιούνται στην μέθοδο updateGrid()
                    Ελέγχει έαν το κελί είναι κάτω απο το γειτονικό
                    * Δηλαδή ελέγχει εαν η λίστα neighbors περιέχει το απο κάτω στοιχείο
                 */
                boolean isCellBelowNeighbor() {
                    return this.neighbors.contains(new Cell(this.x, this.y + 1));
                }
                 /** Χρησιμοποιούνται στην μέθοδο updateGrid()
                     Ελέγχει έαν το κελί είναι δίπλα(δεξιά) απο το γειτονικό
                     * Δηλαδή ελέγχει εαν η λίστα neighbors περιέχει το διπλανό στοιχείο
                  */
                boolean isCellRightNeighbor() {
                    return this.neighbors.contains(new Cell(this.x + 1, this.y));
                }
                /* χρησιμες ισοδυναμίες του Cell*/
                @Override
                public boolean equals(Object other) 
                {
                    if (!(other instanceof Cell)) return false;//εαν το στοιχείο δεν είναι instance του Cell
                    Cell otherCell = (Cell) other;   // να το κάνεις instance με Casting
                    return (this.x == otherCell.x && this.y == otherCell.y); //και να επιστρέψεις τις συντεταγμένες του καινούριου
                }

                // πρέπει να γίνει overridden με την equals
                @Override
                public int hashCode() 
                {
                    // τυχαίο μίγμα κωδικών. Η μέθοδος έχει σχεδιαστεί να είναι μοναδική
                    return this.x + this.y * 256;
                }

            }
            /** Δημιούργησε τον λαβύρινθο από την επάνω αριστερη γωνία*/
            private void generateMaze() 
            {
                generateMaze(0, 0);
            }
            /** Δημιούργει τον λαβύρινθο από τις συντεταγμένες x, y*/
            private void generateMaze(int x, int y) 
            {
                generateMaze(getCell(x, y)); // Δημιουργία απο το Cell
            }
            /**Δημιουργία (τυχαίου) λαβυρίνθου από το Cell*/
            private void generateMaze(Cell startAt) //το startAt είναι αντικείμενο τύπου Cell άρα θα πάρει τιμές(ορίσματα) x,y-
            {
                // Εαν η αρχική θέση είναι κενή μην δημιουργήσεις από εδώ
                if (startAt == null) return;
                startAt.open = false; //Και στη συνέχεια, υποδικνύει ότι το κελί(Cell) είναι κλειστό για δημιουργία
                //Αλλιώς...
                /**Δημιουργία Λίστας τύπου Cell*/
                ArrayList<Cell> cellsList = new ArrayList<>();
                cellsList.add(startAt);//βάλε τις αρχικές συντεταγμένες μες στη λίστα

                while (!cellsList.isEmpty()) //SOS η αρχική λίστα είναι γεμάτη...
                {
                    /**Αντικείμενο τύπου Cell, για μελλοντική αποθήκευση τιμών*/
                    Cell cell;
                    /*Αυτό είναι για να μειώσουμε αλλά όχι για να εξαλείψουμε τελείως τους αριθμούς 
                     */
                   
                    if (random.nextInt(10)==0) //δημιουργία τυχαίων αριθμών μέχρι το 10, εαν ο τυχαίος αριθμός που θα δημιουργηθεί είναι το 0
            //αφαίρεσε από τη λίστα το αντικείμενο της που βρίσκεται σε μια τυχαία θέση ->την θέση αυτή την βρίσκουμε από το μέγεθος της λίστας, και δώσε την τιμή του αντικειμένου που αφαίρεσες στο cell
                        cell = cellsList.remove(random.nextInt(cellsList.size())); 
                    else cell = cellsList.remove(cellsList.size() - 1); //αφαίρεσε από την λίστα το τελευταίο της στοιχείο και αποδωσέτο στο αντικείμενο cell
                    //Λίστα για συλλογή
                    ArrayList<Cell> neighbors = new ArrayList<>();
                    // κελιά(cells) τα οποία θα μπορούσαν να είναι γειτονικά
                    Cell[] potentialNeighbors = new Cell[] //νέος πίνακας τύπου Cell (πίνακας)
                    {
                        getCell(cell.x + 1, cell.y), //κάτω (θέση 0 σε αυτον το πίνακα)
                        getCell(cell.x, cell.y + 1), //δεξιά (θέση 1 σε αυτον το πίνακα)
                        getCell(cell.x - 1, cell.y), //αριστερά (θέση 2 σε αυτον το πίνακα)
                        getCell(cell.x, cell.y - 1) //πάνω (θέση 3 σε αυτον το πίνακα)
                    };
                    for (Cell other : potentialNeighbors) //πάρε τις τριγυρω περιοχές(χωρις εμπόδια) και βάλτες σε μια καινούρια λίστα
                    {
                        //προσπερασέτο εαν έξω, υπάρχει ένα τοίχος ή δεν είναι ανοιχτό
                        if (other==null || other.wall || !other.open) continue;//βγές απο αυτό το σημείο της λούπας και πήγαινε στο επόμενο στοιχείο
                        neighbors.add(other); //πρόσθεσε στη λίστα τα διαθέσημα κελιά, εκτός αυτών εκτος αυτών που είναι τοίχη η κλειστά
                    }
                    if (neighbors.isEmpty()) continue; //βγές απο αυτό το σημείο της λούπας και πήγαινε στο επόμενο. πήγαινε πάλι στο while
                    // πάρε ένα τυχαίο κελί(cell) από τη λίστα και αποδωσέτο στο αντικείμενο selected(το τυχαίο αντικείμενο μπορεί να ειναι το: επάνω,κάτω,δεξιο,αριστερο)
                    Cell selected = neighbors.get(random.nextInt(neighbors.size()));
                    //πρόσθεσε το κελί που επέλεξες  σαν γειτονικό κελί 
                    selected.open = false; //και στη συνεχεια δήλωσε το κελί αυτό κλειστό για παραγωγή
                    cell.addNeighbor(selected);  //με την μεθοδο προσθέτουμε το κελί που επελέγει σε μια λίστα(απλά καλούμε την μέθοδο με το αντικείμενο cell)
                    cellsList.add(cell); //βάλε στη λίστα τις συντεταγμένες (αντικείμενο) , οι συντεταγμενες την πρώτη φορα θα είναι οι αρχικες σντεταγμένες του 1 κόμβου
                    cellsList.add(selected);  //αυτές οι συντεταγμένες είναι μια τυχαία γειτονική συντεταγμένη γύρω από τον αρχικό κόμβο
                }
                updateGrid();
            }
            /** Χρησιμοποιείται για να πάρει ένα κελί(κόμβο) από τις συντεταγμένες x, y; Επιστρέφει null όταν είναι εκτός ορίων*/
            public Cell getCell(int x, int y) {
                try {
                    return cells[x][y];
                } catch (ArrayIndexOutOfBoundsException e) { // catch εκτός ορίου
                    return null;
                }
            }
            // ζωγράφισε τον λαβύρινθο
            public void updateGrid() 
            {
                char backChar = ' ', wallChar = 'X', cellChar = ' ';
                // γέμισμα του background
                for (int x = 0; x < gridDimensionX; x ++) 
                {
                    for (int y = 0; y < gridDimensionY; y ++) 
                    {
                        mazeGrid[x][y] = backChar;
                    }
                }
                // χτίσε τους τοίχους
                for (int x = 0; x < gridDimensionX; x ++) 
                {
                    for (int y = 0; y < gridDimensionY; y ++) 
                    {
                        if (x % 2 == 0 || y % 2 == 0)
                            mazeGrid[x][y] = wallChar;
                    }
                }
                // κάνει μια αναπαράσταση
                for (int x = 0; x < dimensionX; x++) 
                {
                    for (int y = 0; y < dimensionY; y++) 
                    {
                        Cell current = getCell(x, y);
                        int gridX = x * 2 + 1, gridY = y * 2 + 1;
                        mazeGrid[gridX][gridY] = cellChar;
                        if (current.isCellBelowNeighbor()) 
                        {
                            mazeGrid[gridX][gridY + 1] = cellChar;
                        }
                        if (current.isCellRightNeighbor()) 
                        {
                            mazeGrid[gridX + 1][gridY] = cellChar;
                        }
                    }
                }
                
                 // Δημιουργούμε ένα καθαρό πλέγμα ...
                searching = false;
                endOfSearch = false;
                fillGrid();
                // ... και αντιγράφουμε μέσα του τις θέσεις των εμποδίων
                // που δημιούργησε ο αλγόριθμος κατασκευής του λαβύρινθου
                for (int x = 0; x < gridDimensionX; x++) 
                {
                    for (int y = 0; y < gridDimensionY; y++) 
                    {
                        if (mazeGrid[x][y] == wallChar && grid[x][y] != ROBOT && grid[x][y] != TARGET)
                        {
                            grid[x][y] = OBST;
                        }
                    }
                }
            }
        } // τέλος της βοηθητικής κλάσης MyMaze
        
        /**
         **********************************************************
         *          Σταθερές της κλάσης MazePanel
         **********************************************************
         */
                    
            /**κενό κελί */    
          private final static int EMPTY    = 0;  
            /**κελί με εμπόδιο */    
          private final static int OBST     = 1;  
            /**η θέση του ρομπότ(αρχική θέση) */    
          private final static int  ROBOT    = 2;  
            /**η θέση του στόχου (τελική θέση) */    
          private final static int  TARGET   = 3;  
            /**κελιά του μετώπου αναζήτησης (ΑΝΟΙΚΤΈΣ καταστάσεις)(μπλέ) */    
          private final static int   FRONTIER = 4;   
            /**(Σιάν)κελιά κλειστών καταστάσεων (οι περιοχές που έχουν εξερευνηθεί και έχουν μπει στην ΚΛΕΙΣΤΗ λίστα) */    
          private final static int   CLOSED   = 5;   
            /**κελιά που σχηματίζουν τη διαδρομή ρομπότ-στόχος (η τελική διαδρομή)(Υπολογίζεται από την τελευταία θέση(στόχος) και πρός τα πίσω βρίσκοντας την πιο σύντομη διαδρομή των κλειστών καταστάσεων) */    
          private final static int   ROUTE    = 6;   
          
          public static int num_DFS = -1;
        
         /** Μηνύματα προς τον χρήστη*/
        private final static String
            msgDrawAndSelect =
                "\"Σχεδιάστε\" εμπόδια και επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση'",
            msgSelectStepByStepEtc =
                "Επιλέξτε 'Βήμα-Βήμα' ή 'Κίνηση' ή 'Καθάρισμα'",
            msgNoSolution =
                "Δεν υπάρχει διαδρομή για τον στόχο !!!";

        /**
         **********************************************************
         *          Μεταβλητές της κλάσης MazePanel
         **********************************************************
         */
        
         /** (βέλη) Spinners για την είσοδο του αριθμού των γραμμών και των στηλών*/
        JSpinner rowsSpinner, columnsSpinner;
        
        /** Ο αρχικός αριθμός των γραμμών του πλέγματος(grid)*/
        int rows    = 5;          
        /** Ο αρχικός αριθμός των στηλών του πλέγματος(grid)*/
        int   columns = 5;         
        /** Οι διαστάσεις του εκάστοτε κελιού(Cell) σε pixels, εξαρτάται πάντα από τον αριθμό των γραμμών*/
        int squareSize = 500/rows;          
        /** Το μέγεθος της μύτης του βέλους που δείχνει το προκάτοχο κελί*/
        int arrowSize = squareSize/2; 
        /** το σύνολο ανοικτών καταστάσεων */
        ArrayList<Cell> openSet   = new ArrayList();
        /**το σύνολο κλειστών καταστάσεων */
        ArrayList<Cell> closedSet = new ArrayList();                                                        
        /**η αρχική θέση του ρομπότ */
        Cell robotStart; 
        /**η θέση του στόχου */
        Cell targetPos;  
        /**μήνυμα προς τον χρήστη */
        JLabel message;         
        /**Τα Βασικά κουμπιά */
        JButton resetButton, mazeButton, clearButton, realTimeButton, stepButton, animationButton;
        /**Η λίστα που θα μας χρειαστεί για να ζωγραφήσουμε τους αριθμούς μέσα στα κελιά
         Λίστα με τις συντεταγμένες*/
        ArrayList<Cell> array = new ArrayList<>();
        
        /**τα κουμπιά για την επιλογή του αλγόριθμου */
        JRadioButton dfs, bfs, aStar;
        
        /**ο slider για την ρύθμιση της ταχύτητας του animation */
        JSlider slider;        
        /**Επιτρέπονται διαγώνιες κινήσεις; */
        JCheckBox diagonal;
        /**Σχεδίαση βελών προς προκατόχους */
        JCheckBox drawArrows;
        /**Σχεδίαση αριθμών*/
        JCheckBox drawNumbers;
        /** Το πλέγμα(grid)*/
        int[][] grid;  
        /** Η λύση εμφανίζεται αμέσως*/
        boolean realTime;  
        /** flag ότι βρέθηκε ο στόχος*/
        boolean found;
        /** flag ότι η αναζήτηση είναι σε εξέλιξη*/
        boolean searching;   
        /** flag ότι η αναζήτηση έφθασε στο τέρμα*/
        boolean endOfSearch;
        /** ο χρόνος της καθυστέρησης σε msec του animation*/
        int delay;          
        /** ο αριθμός των κόμβων που έχουν αναπτυχθεί*/
        int expanded;     
        
        /** το αντικείμενο που ελέγχει το animation*/
        RepaintAction action = new RepaintAction();
        
        /** ο Timer που ρυθμίζει την ταχύτητα εκτέλεσης του animation*/
        Timer timer;
      
        /**
         * Ο δημιουργός του panel
         * @param width το πλάτος του panel.
         * @param height το ύψος panel.
         */
        public MazePanel(int width, int height) 
        {
      
            setLayout(null);
            
            MouseHandler listener = new MouseHandler();
            addMouseListener(listener);
            addMouseMotionListener(listener);

            setBorder(BorderFactory.createMatteBorder(2,2,2,2,Color.blue));

            setPreferredSize( new Dimension(width,height) );

            grid = new int[rows][columns];

            // Δημιουργούμε τα περιεχόμενα του panel

            message = new JLabel(msgDrawAndSelect, JLabel.CENTER);
            message.setForeground(Color.blue);
            message.setFont(new Font("Helvetica",Font.PLAIN,16));

            JLabel rowsLbl = new JLabel("Πλήθος γραμμών (5-83):", JLabel.RIGHT);
            rowsLbl.setFont(new Font("Helvetica",Font.PLAIN,13));

            SpinnerModel rowModel = new SpinnerNumberModel(5, //αρχική τιμή
                                       5,  //min
                                       83, //max
                                       1); //step
            rowsSpinner = new JSpinner(rowModel);
 
            JLabel columnsLbl = new JLabel("Πλήθος στηλών (5-83):", JLabel.RIGHT);
            columnsLbl.setFont(new Font("Helvetica",Font.PLAIN,13));

            SpinnerModel colModel = new SpinnerNumberModel(5, //αρχική τιμή
                                       5,  //min
                                       83, //max
                                       1); //step
            columnsSpinner = new JSpinner(colModel);

            resetButton = new JButton("Νέο πλέγμα");
            resetButton.addActionListener(new ActionHandler());
            resetButton.setBackground(Color.lightGray);
            resetButton.setToolTipText
                    ("Καθαρίζει και επανασχεδιάζει το πλέγμα με βάση τις δοθείσες διαστάσεις του");
            resetButton.addActionListener(this::resetButtonActionPerformed);

            mazeButton = new JButton("Λαβύρινθος");
            mazeButton.addActionListener(new ActionHandler());
            mazeButton.setBackground(Color.lightGray);
            mazeButton.setToolTipText
                    ("Δημιουργεί έναν τυχαίο λαβύρινθο");
            mazeButton.addActionListener(this::mazeButtonActionPerformed);

            clearButton = new JButton("Καθάρισμα");
            clearButton.addActionListener(new ActionHandler());
            clearButton.setBackground(Color.lightGray);
            clearButton.setToolTipText
                    ("Πρώτο κλικ: καθάρισμα αναζήτησης, Δεύτερο κλικ: καθάρισμα εμποδίων");

            realTimeButton = new JButton("Σε πραγματικό χρόνο");
            realTimeButton.addActionListener(new ActionHandler());
            realTimeButton.setBackground(Color.lightGray);
            realTimeButton.setToolTipText
                    ("Η θέση των εμποδίων, του ρομπότ και του στόχου μπορεί να αλλάξει όταν η αναζήτηση βρίσκεται σε εξέλιξη");

            stepButton = new JButton("Βήμα - Βήμα");
            stepButton.addActionListener(new ActionHandler());
            stepButton.setBackground(Color.lightGray);
            stepButton.setToolTipText
                    ("Η αναζήτηση γίνεται βήμα-βήμα για κάθε κλικ");

            animationButton = new JButton("Κίνηση");
            animationButton.addActionListener(new ActionHandler());
            animationButton.setBackground(Color.lightGray);
            animationButton.setToolTipText
                    ("Η αναζήτηση γίνεται αυτόματα");

            JLabel velocity = new JLabel("Ταχύτητα", JLabel.CENTER);
            velocity.setFont(new Font("Helvetica",Font.PLAIN,10));
            
            slider = new JSlider(0,1000,500); //αρχική τιμή καθυστέρησης 500 msec
            slider.setToolTipText
                    ("Ρυθμίζει την καθυστέρηση σε κάθε βήμα (0 μέχρι 1 sec)");
            
            delay = 1000-slider.getValue();
            slider.addChangeListener((ChangeEvent evt) -> {
                JSlider source = (JSlider)evt.getSource();
                if (!source.getValueIsAdjusting()) {
                    delay = 1000-source.getValue();
                }
            });
            
            // ButtonGroup που συγχρονίζει τα πέντε RadioButtons
            // που επιλέγουν τον αλγόριθμο, έτσι ώστε ένα μόνο από
            // αυτά να μπορεί να επιλεγεί ανά πάσα στιγμή
            ButtonGroup algoGroup = new ButtonGroup();

            dfs = new JRadioButton("DFS");
            dfs.setToolTipText("Αλγόριθμος αναζήτησης πρώτα σε βάθος");
            algoGroup.add(dfs);
            dfs.addActionListener(new ActionHandler());

            bfs = new JRadioButton("BFS");
            bfs.setToolTipText("Αλγόριθμος αναζήτησης πρώτα σε πλάτος");
            algoGroup.add(bfs);
            bfs.addActionListener(new ActionHandler());

            aStar = new JRadioButton("A*");
            aStar.setToolTipText("Αλγόριθμος αναζήτησης Α*");
            algoGroup.add(aStar);
            aStar.addActionListener(new ActionHandler());   

            JPanel algoPanel = new JPanel();
            algoPanel.setBorder(javax.swing.BorderFactory.
                    createTitledBorder(javax.swing.BorderFactory.createEtchedBorder(),
                    "Αλγόριθμοι", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION,
                    javax.swing.border.TitledBorder.TOP, new java.awt.Font("Helvetica", 0, 14)));
            
            dfs.setSelected(true);  // Αρχικά ο DFS έχει επιλεγεί
            
            diagonal = new
                    JCheckBox("Διαγώνιες κινήσεις");
            diagonal.setToolTipText("Να επιτρέπονται και διαγώνιες κινήσεις");

            drawArrows = new
                    JCheckBox("Βέλη προς προκατόχους");
            drawArrows.setToolTipText("Σχεδίαση βελών προς προκατόχους καταστάσεις");
            
            drawNumbers = new
                    JCheckBox("Αρίθμηση τετραγώνων");
            drawNumbers.setToolTipText("Η σειρά των τετραγώνων που εξετάζει ο πράκτορας κάθε φορά");

            JLabel robot = new JLabel("Ρομπότ", JLabel.CENTER);
            robot.setForeground(Color.green);
            robot.setFont(new Font("Helvetica",Font.PLAIN,14));

            JLabel target = new JLabel("Στόχος", JLabel.CENTER);
            target.setForeground(Color.red);
            target.setFont(new Font("Helvetica",Font.PLAIN,14));
         
            JLabel frontier = new JLabel("Μέτωπο", JLabel.CENTER);
            frontier.setForeground(Color.blue);
            frontier.setFont(new Font("Helvetica",Font.PLAIN,14));

            JLabel closed = new JLabel("Κλειστές", JLabel.CENTER);
            closed.setForeground(Color.CYAN);
            closed.setFont(new Font("Helvetica",Font.PLAIN,14));

            JButton aboutButton = new JButton("Σχετικά με το Maze");
            aboutButton.addActionListener(new ActionHandler());
            aboutButton.setBackground(Color.lightGray);

            // προσθέτουμε τα περιεχόμενα στο panel
            add(message);
            add(rowsLbl);
            add(rowsSpinner);
            add(columnsLbl);
            add(columnsSpinner);
            add(resetButton);
            add(mazeButton);
            add(clearButton);
            add(realTimeButton);
            add(stepButton);
            add(animationButton);
            add(velocity);
            add(slider);
            add(dfs);
            add(bfs);
            add(aStar);               
            add(algoPanel);
            add(diagonal);
            add(drawArrows);
            add(drawNumbers);
            add(robot);
            add(target);
            add(frontier);
            add(closed);
            add(aboutButton);

            // ρυθμίζουμε τα μεγέθη και τις θέσεις τους
            message.setBounds(0, 515, 500, 23);
            rowsLbl.setBounds(520, 5, 130, 25);
            rowsSpinner.setBounds(655, 5, 35, 25);
            columnsLbl.setBounds(520, 35, 130, 25);
            columnsSpinner.setBounds(655, 35, 35, 25);
            resetButton.setBounds(520, 65, 170, 25);
            mazeButton.setBounds(520, 95, 170, 25);
            clearButton.setBounds(520, 125, 170, 25);
            realTimeButton.setBounds(520, 155, 170, 25);
            stepButton.setBounds(520, 185, 170, 25);
            animationButton.setBounds(520, 215, 170, 25);
            velocity.setBounds(520, 245, 170, 10);
            slider.setBounds(520, 255, 170, 25);
            dfs.setBounds(530, 300, 70, 25);
            bfs.setBounds(600, 300, 70, 25);
            aStar.setBounds(530, 325, 70, 25);                  
            algoPanel.setLocation(520,280);
            algoPanel.setSize(170, 80);
            diagonal.setBounds(520, 365, 170, 25);
            drawArrows.setBounds(520, 390, 170, 25);
            drawNumbers.setBounds(520, 415, 170, 25);
            robot.setBounds(520, 450, 80, 25);
            target.setBounds(605, 450, 80, 25);
            frontier.setBounds(520, 470, 80, 25);
            closed.setBounds(605, 470, 80, 25);
            aboutButton.setBounds(520, 510, 170, 25);

            // δημιουργούμε τον timer
            timer = new Timer(delay, action);
            
            // δίνουμε στα κελιά του πλέγματος αρχικές τιμές
            // εδώ γίνεται και το πρώτο βήμα των αλγόριθμων
            fillGrid1();

        } // τέλος του MazePanel

        
  

        /**
         * Λειτουργία που εκτελείται αν ο χρήστης πιέσει το κουμπί "Νέο Πλέγμα"
         */
        private void resetButtonActionPerformed(java.awt.event.ActionEvent evt) {                                           
            realTime = false;
            realTimeButton.setEnabled(true);
            realTimeButton.setForeground(Color.black);
            stepButton.setEnabled(true);
            animationButton.setEnabled(true);
            slider.setEnabled(true);
            initializeGrid(false);
        } // τέλος λειτουργίας resetButtonActionPerformed()
    
        /**
         * Λειτουργία που εκτελείται αν ο χρήστης πιέσει το κουμπί "Λαβύρινθος"
         */
        private void mazeButtonActionPerformed(java.awt.event.ActionEvent evt) {
            realTime = false;
            realTimeButton.setEnabled(true);
            realTimeButton.setForeground(Color.black);
            stepButton.setEnabled(true);
            animationButton.setEnabled(true);
            slider.setEnabled(true);
            initializeGrid(true);
        } // τελος λειτουργίας mazeButtonActionPerformed()
    
        /**
         * Δημιουργεί ένα νέο καθαρό πλέγμα ή ένα νέο λαβύρινθο
         */
        private void initializeGrid(Boolean makeMaze) 
        {                                           
            rows    = (int)(rowsSpinner.getValue()); //παίρνει την αρχική τιμη των γραμμών που έχει βάλει ο χρήστης στο textbox και την αποδιδει στη rows
            columns = (int)(columnsSpinner.getValue());
            squareSize = 500/(rows > columns ? rows : columns);
            arrowSize = squareSize/2;
            //Ο λαβύρινθος πρέπει να έχει έναν περιττό αριθμό γραμμών και στηλών (στην περίπτωση που θα πατήσουμε το κουμπί "Δημιουργια λαβυρινθου"
            if (makeMaze && rows % 2 == 0) {
                rows -= 1;
            }
            if (makeMaze && columns % 2 == 0) {
                columns -= 1;
            }
            grid = new int[rows][columns];
            robotStart = new Cell(rows - 2,0);        //η αρχική θέση του ρομπότ 
            targetPos = new Cell(rows - 2,columns-2);
            dfs.setEnabled(true);
            dfs.setSelected(true);
            bfs.setEnabled(true);
            aStar.setEnabled(true);                  
            diagonal.setSelected(false);
            diagonal.setEnabled(true);
            drawArrows.setSelected(false);
            drawArrows.setEnabled(true);
            drawNumbers.setSelected(false);
            drawNumbers.setEnabled(true);
            slider.setValue(500);
            if (makeMaze) {
                MyMaze maze = new MyMaze(rows/2,columns/2); // εαν εχει παρει τιμή αληθείας δημιουργεί εναν τυχαίο λαβύρυνθο
            } else {
                fillGrid(); // αλλιως δημιουργεί ένα κενό πλέγμα (με άσπρα κελιά)
            }
        } // τέλος λειτουργίας initializeGrid()
   
        /**
         * Επεκτείνει ένα κόμβο και δημιουργεί τους διαδόχους του
         */
        private void expandNode(){            //<<<<<<<<<<<<<<<<<<<<<<--------------------------------------------πηγαίνει στον επόμενο κόμβο
                Cell current;
                if (dfs.isSelected() || bfs.isSelected()) {
                    // Εδώ έχουμε το 3ο βήμα των αλγόριθμων DFS και BFS
                    // 3. Αφαίρεσε την πρώτη κατάσταση Si, από τις ΑΝΟΙΚΤΕΣ .... (Η πρώτη(αρχική) κατάσταση θα είναι πάντα η θέση του ΡΟΜΠΟΤ)
                    current = openSet.remove(0);      //<<<<<<<<<<<---------------------------------στοίβα
                } else //εαν ο Α* έχει επιλεγεί
                { 
                    // Εδώ έχουμε το 3ο βήμα του αλγορίθμου Α*
                    // 3. Αφαίρεσε την κατάσταση Si, από την λίστα ΑΝΟΙΚΤΕΣ,
                    //    για την οποία f(Si) <= f(Sj) για όλες τις άλλες
                    //    ανοικτές καταστάσεις Sj ...
                    // (ταξινομούμε πρώτα τη λίστα ΑΝΟΙΚΤΕΣ κατά αύξουσα σειρά ως προς f)
                    Collections.sort(openSet, new CellComparatorByF());//ταξινομεί την openset με βάση το f(την απόσταση δηλ)
                    current = openSet.remove(0);
                }
                //... και πρόσθεσέ την πρώτη κατάσταση στις ΚΛΕΙΣΤΕΣ.
                closedSet.add(0,current);               
              
                array.add(current);// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<--------------------ΤΗΝ ΧΡΕΙΑΖΟΜΑΙ ΓΙΑ ΝΑ ΖΩΓΡΑΦΙΣΩ ΤΑ ΝΟΥΜΕΡΑ    
                
                // Ενημέρωσε το χρώμα του κελιού
                grid[current.row][current.col] = CLOSED;
                // Αν ο επιλεγμένος κόμβος είναι ο στόχος ...
                if (current.row == targetPos.row && current.col == targetPos.col) 
                {
                    // ... τότε τερμάτισε κλπ
                    Cell last = targetPos;
                    last.prev = current.prev;
                    closedSet.add(last);  //βάζει τον τελικό στόχο μέσα στην κλειστή λίστα για μετέπειτα επεξεργασία
                    found = true;
                    return;
                }
                // Καταμετρούμε τους κόμβους που έχουμε αναπτύξει.
                expanded++;
                // Εδώ έχουμε το 4ο βήμα των αλγόριθμων
                // 4. Δημιούργησε τις διαδόχους της Si, με βάση τις ενέργειες που μπορούν
                //    να εφαρμοστούν στην Si.
                //    Η κάθε διάδοχος έχει ένα δείκτη προς την Si, ως την προκάτοχό της.
                //    Στην περίπτωση των αλγόριθμων DFS και BFS οι διάδοχοι δεν πρέπει
                //    να ανήκουν ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ.
                ArrayList<Cell> succesors;
                //αποδίδει τα αποτελέσματα της createSuccesors στην λίστα succesors(Τα αποτελέσματα είναι μια λίστα με όνομα temp), τα οποία στοιχεία της τα βάζει με ανάποδη σειρα στους succesors
                succesors = createSuccesors(current);   //<<<<<<<<<<<<<<<<<<-----------------------------βρες τα γειτονικά κελια(δημιουργια διαδόχων)
                // Εδώ έχουμε το 5ο βήμα των αλγόριθμων
                // 5. Για κάθε διάδοχο της Si, ...
                succesors.stream().forEach((cell) -> {  //<<<<<--για κάθε γειτονικό κελί...
                    // ... αν τρέχουμε τον DFS ...
                    if (dfs.isSelected()) {
                        //  ... πρόσθεσε τον διάδοχο στην αρχή της λίστας ΑΝΟΙΚΤΕΣ
                        openSet.add(0, cell); //<<<<<<<<<----------------------------------------------------------------στοίβα (DFS)
                        // Ενημέρωσε το χρώμα του κελιού
                        grid[cell.row][cell.col] = FRONTIER;
                        //  ... αν τρέχουμε τον ΒFS ...
                    } else if (bfs.isSelected()){
                        //  ... πρόσθεσε τον διάδοχο στο τέλος της λίστας ΑΝΟΙΚΤΕΣ
                        openSet.add(cell);  //<<<<<<<<<----------------------------------------------------------------ουρα   (BFS)
                        //  Ενημέρωσε το χρώμα του κελιού
                        grid[cell.row][cell.col] = FRONTIER;
                        //  ... αν τρέχουμε τον αλγόριθμο Α* (Βήμα 5 αλγόριθμου Α*) ...
                    } else if (aStar.isSelected())
                    {
                        // ... υπολόγισε την τιμή f(Sj)...
                        int dxg = current.col-cell.col;//διαφορά(απόσταση) της τρέχων θέσης από την γειτονική
                        int dyg = current.row-cell.row;
                        int dxh = targetPos.col-cell.col; //διαφορά(απόσταση) της τελικής θέσης από την γειτονική
                        int dyh = targetPos.row-cell.row;
                        if (diagonal.isSelected())
                        {
                            
                            if(Math.abs(dxg) == 1 && Math.abs(dyg) == 1) //εαν η απόσταση των x και y είναι 1(δηλαδή όταν έχουμε διαγώνιο κελι...)
                            {
                                cell.g = current.g +(int)((double)2*Math.sqrt(dxg*dxg + dyg*dyg)); //το κόστος μετάβασης θα είναι 2
                                //cell.h = (int)((double)Math.sqrt(dxh*dxh + dyh*dyh)); //απόσταση  μεχρι το τελος
                            }
                            else //εαν η απόσταση χ ή η απόσταση y δεν είναι 1 (εαν δεν έχουμε δηλαδή διαγώνιο κελί)
                            {
                                cell.g = current.g + (Math.abs(dxg)+Math.abs(dyg)); //το κόστος μετάβασης θα είναι 1
                                //cell.h = Math.abs(dxh)+Math.abs(dyh);
                            }                   
                            cell.h = Math.abs(dxh)+Math.abs(dyh);
                             

                             // Με διαγώνιες κινήσεις υπολογίζουμε
                             // το 2-πλάσιο των ευκλείδιων αποστάσεων
                           
                            //Το g του γειτονικού κόμβου είναι το g του τρέχων + το g της διαγωνιου
                            //cell.g = current.g+(int)((double)2*Math.sqrt(dxg*dxg + dyg*dyg)); 
                            
                            //η απόσταση του γειτονικού κελιού μέχρι τον τερματισμό(cell.h)
                            //cell.h = (int)((double)2*Math.sqrt(dxh*dxh + dyh*dyh));
                            
                            
                        } else   // Χωρίς διαγώνιες κινήσεις υπολογίζουμε
                        {                          
                            // τις αποστάσεις Manhattan                           
                            cell.g = current.g+Math.abs(dxg)+Math.abs(dyg);
                            
                            //η απόσταση του γειτονικού κελιού μέχρι τον τερματισμό(cell.h)
                            cell.h = Math.abs(dxh)+Math.abs(dyh); 
                        }
                        cell.f = cell.g+cell.h; //το χρησιμοποιούμε για να επιλέξουμε τον συντομότερο κόμβο κατά την αναζήτηση
                        // ... αν η Sj(το γειτονικό κελί) δεν ανήκει ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        int openIndex   = isInList(openSet,cell);
                        int closedIndex = isInList(closedSet,cell);
                        if (openIndex == -1 && closedIndex == -1) 
                        {
                            // ... τότε πρόσθεσε την Sj στις ΑΝΟΙΚΤΕΣ ...
                            // ... με τιμή αξιολόγησης f(Sj)
                            openSet.add(cell);
                            // Ενημέρωσε το χρώμα του κελιού
                            grid[cell.row][cell.col] = FRONTIER;
                            // Αλλιώς ...
                        } else //Εαν το γειτονικό κελί υπάρχει η στις ανοιχτές η και στις κλειστές
                        {
                            // ... αν ανήκει στις ΑΝΟΙΚΤΕΣ, τότε ...
                            if (openIndex > -1){
                                // ... σύγκρινε την νέα τιμή αξιολόγισής της με την παλαιά.
                                // Αν παλαιά <= νέα ...
                                if (openSet.get(openIndex).f <= cell.f) {  //πρέπει να παίρνουμε πάντα την μικρότερη τιμή
                                    // ... απόβαλε το νέο κόμβο με την κατάσταση Sj
                                    // (δηλαδή μην κάνεις τίποτε για αυτόν τον κόμβο).
                                    // Διαφορετικά, ...
                                } else {
                                    // ... αφαίρεσε το στοιχείο (Sj,παλαιά) από τη λίστα
                                    // στην οποία ανήκει ...
                                    openSet.remove(openIndex);
                                    // ... και πρόσθεσε το στοιχείο (Sj,νέα) στις ΑΝΟΙΚΤΕΣ
                                    openSet.add(cell);
                                    // Ενημέρωσε το χρώμα του κελιού
                                    grid[cell.row][cell.col] = FRONTIER;
                                }
                               // ... αν ανήκε στις ΚΛΕΙΣΤΕΣ, τότε ...
                            } else {
                                // ... σύγκρινε την νέα τιμή αξιολόγισής της με την παλαιά.
                                // Αν παλαιά <= νέα ...
                                if (closedSet.get(closedIndex).f <= cell.f) {
                                    // ... απόβαλε το νέο κόμβο με την κατάσταση Sj
                                    // (δηλαδή μην κάνεις τίποτε για αυτόν τον κόμβο).
                                    // Διαφορετικά, ...
                                } else {
                                    // ... αφαίρεσε το στοιχείο (Sj,παλαιά) από τη λίστα
                                    // στην οποία ανήκει ...
                                    closedSet.remove(closedIndex);
                                    // ... και πρόσθεσε το στοιχείο (Sj,νέα) στις ΑΝΟΙΚΤΕΣ
                                    openSet.add(cell);
                                    // Ενημέρωσε το χρώμα του κελιού
                                    grid[cell.row][cell.col] = FRONTIER;
                                }
                            }
                        }
                    }
                });
            
        } //Τέλος της λειτουργίας expandNode()
        
        /**
         * Δημιουργεί τους διαδόχους μιας κατάστασης/κελιού
         * 
         * @param current το κελί του οποίου ζητούμε τους διαδόχους         
         * @return οι διάδοχοι του κελιού με μορφή λίστας
         */
        private ArrayList<Cell> createSuccesors(Cell current){ //<<<<<<<<<<<<<<-----------------------βρες τα γειττονικα κελια
            int r = current.row;
            int c = current.col;
            // Δημιουργούμε μια κενή λίστα για τους διαδόχους του τρέχοντος κελιού.
            ArrayList<Cell> temp = new ArrayList<>();
            // Με διαγώνιες κινήσεις η προτεραιότητα είναι:
            // 1:Πάνω 2:Πάνω-δεξιά 3:Δεξιά 4:Κάτω-δεξιά
            // 5:Κάτω 6:Κάτω-αριστερά 7:Αριστερά 8:Πάνω-αριστερά
            
            // Χωρίς διαγώνιες κινήσεις η προτεραιότητα είναι:
            // 1:Πάνω 2:Δεξιά 3:Κάτω 4:Αριστερά 
            
            // Αν δεν βρισκόμαστε στο πάνω όριο του πλέγματος (εαν δεν βρισκόμαστε δηλ στην γραμμή 0)
            // και το πάνω κελί δεν είναι εμπόδιο ...
            if (r > 0 && grid[r-1][c] != OBST &&
                    // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ) 
                    // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                    ((aStar.isSelected()) ? true :
                          isInList(openSet,new Cell(r-1,c)) == -1 && //εαν το κελί δεν ανήκει στην ανοιχτη λίστα
                          isInList(closedSet,new Cell(r-1,c)) == -1)) { //και εαν το κελί δεν ανήκει στην κλειστή λίστα
                          Cell cell = new Cell(r-1,c);  //δημιούργησε ένα κελί (cell)  και πέρνα του τις συντεταγμενες(του πάνω κόμβου)                
                
                        // ... ενημέρωσε τον δείκτη του πάνω κελιού να δείχνει το τρέχον ...
                        cell.prev = current; //στο αντικειμενο(μεταβλητη) prev αποθηκευουμε την τρέχων κατάσταση <<<<<<<<<<<<<<<<<<<<-------------αποθηκεύουμε το τρέχον κελί στη μεταβλητή prev
                        // ... και πρόσθεσε το πάνω κελί στους διαδόχους του τρέχοντος(δηλαδή το πάνω κελί θα είναι διάδοχος του τρέχων κελιου). 
                        temp.add(cell); //<<<<<<<<<<<<<<<<<<<<<<<<----------------------------------------Αποθηκεύουμε το πάνω κελι 
                 
            }
            if (diagonal.isSelected())
            {
                // Αν δεν βρισκόμαστε ούτε στο πάνω ούτε στο δεξιό όριο του πλέγματος
                // και το πάνω-δεξί κελί δεν είναι εμπόδιο ...
                if (r > 0 && c < columns-1 && grid[r-1][c+1] != OBST &&
                        // ... και ένα από τα πάνω ή δεξιό κελιά δεν είναι εμπόδια ...
                        // (επειδή δεν είναι λογικό να επιτρέψουμε να περάσει
                        //  το ρομπότ από μία σχισμή)                        
                        (grid[r-1][c] != OBST || grid[r][c+1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α*) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected()) ? true :
                              isInList(openSet,new Cell(r-1,c+1)) == -1 &&
                              isInList(closedSet,new Cell(r-1,c+1)) == -1)) {
                              Cell cell = new Cell(r-1,c+1);
                    
                        // ... ενημέρωσε τον δείκτη του πάνω-δεξιού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                        // ... και πρόσθεσε το πάνω-δεξί κελί στους διαδόχους του τρέχοντος. 
                        temp.add(cell);
                    
                }
            }
            // Αν δεν βρισκόμαστε στο δεξί όριο του πλέγματος
            // και το δεξί κελί δεν είναι εμπόδιο ...
            if (c < columns-1 && grid[r][c+1] != OBST &&
                // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ) 
                // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                ((aStar.isSelected())? true :
                      isInList(openSet,new Cell(r,c+1)) == -1 &&
                      isInList(closedSet,new Cell(r,c+1)) == -1)) {
                      Cell cell = new Cell(r,c+1);

                 // ... ενημέρωσε τον δείκτη του δεξιού κελιού να δείχνει το τρέχον ...
                cell.prev = current;
                // ... και πρόσθεσε το δεξί κελί στους διαδόχους του τρέχοντος. 
                temp.add(cell);
                
            }
            if (diagonal.isSelected())
            {
                // Αν δεν βρισκόμαστε ούτε στο κάτω ούτε στο δεξιό όριο του πλέγματος
                // και το κάτω-δεξί κελί δεν είναι εμπόδιο ...
                if (r < rows-1 && c < columns-1 && grid[r+1][c+1] != OBST &&
                        // ... και ένα από τα κάτω ή δεξιό κελιά δεν είναι εμπόδια ...
                        (grid[r+1][c] != OBST || grid[r][c+1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α*) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected()) ? true :
                              isInList(openSet,new Cell(r+1,c+1)) == -1 &&
                              isInList(closedSet,new Cell(r+1,c+1)) == -1)) {
                              Cell cell = new Cell(r+1,c+1);
                    
                         // ... ενημέρωσε τον δείκτη του κάτω-δεξιού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                       // ... και πρόσθεσε το κάτω-δεξί κελί στους διαδόχους του τρέχοντος.
                        temp.add(cell);
                    
                }
            }
            // Αν δεν βρισκόμαστε στο κάτω όριο του πλέγματος
            // και το κάτω κελί δεν είναι εμπόδιο ...
            if (r < rows-1 && grid[r+1][c] != OBST &&
                     // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α*) 
                    // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                    ((aStar.isSelected()) ? true :
                          isInList(openSet,new Cell(r+1,c)) == -1 &&
                          isInList(closedSet,new Cell(r+1,c)) == -1)) {
                          Cell cell = new Cell(r+1,c);
                
                    // ... ενημέρωσε τον δείκτη του κάτω κελιού να δείχνει το τρέχον ...
                    cell.prev = current;
                    // ... και πρόσθεσε το κάτω κελί στους διαδόχους του τρέχοντος. 
                    temp.add(cell);
                
            }
            if (diagonal.isSelected())
            {
                // Αν δεν βρισκόμαστε ούτε στο κάτω ούτε στο αριστερό όριο του πλέγματος
                // και το κάτω-αριστερό κελί δεν είναι εμπόδιο ...
                if (r < rows-1 && c > 0 && grid[r+1][c-1] != OBST &&
                        // ... και ένα από τα κάτω ή αριστερό κελιά δεν είναι εμπόδια ...
                        (grid[r+1][c] != OBST || grid[r][c-1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected()) ? true :
                              isInList(openSet,new Cell(r+1,c-1)) == -1 &&
                              isInList(closedSet,new Cell(r+1,c-1)) == -1)) {
                              Cell cell = new Cell(r+1,c-1);
                  
                        // ... ενημέρωσε τον δείκτη του κάτω-αριστερού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                        // ... και πρόσθεσε το κάτω-αριστερό κελί στους διαδόχους του τρέχοντος.
                        temp.add(cell);
                    
                }
            }
            // Αν δεν βρισκόμαστε στο αριστερό όριο του πλέγματος
            // και το αριστερό κελί δεν είναι εμπόδιο ...
            if (c > 0 && grid[r][c-1] != OBST && 
                    // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α* ) 
                    // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                    ((aStar.isSelected()) ? true :
                          isInList(openSet,new Cell(r,c-1)) == -1 &&
                          isInList(closedSet,new Cell(r,c-1)) == -1)) {
                          Cell cell = new Cell(r,c-1);
               
                   // ... ενημέρωσε τον δείκτη του αριστερού κελιού να δείχνει το τρέχον ...
                    cell.prev = current;
                    // ... και πρόσθεσε το αριστερό κελί στους διαδόχους του τρέχοντος.
                    temp.add(cell);
                
            }
            if (diagonal.isSelected())
            {
                // Αν δεν βρισκόμαστε ούτε στο πάνω ούτε στο αριστερό όριο του πλέγματος
                // και το πάνω-αριστερό κελί δεν είναι εμπόδιο ...
                if (r > 0 && c > 0 && grid[r-1][c-1] != OBST &&
                        // ... και ένα από τα πάνω ή αριστερό κελιά δεν είναι εμπόδια ...
                        (grid[r-1][c] != OBST || grid[r][c-1] != OBST) &&
                        // ... και (στην περίπτωση μόνο που δεν εκτελούμε τον Α*) 
                        // δεν ανήκει ήδη ούτε στις ΑΝΟΙΚΤΕΣ ούτε στις ΚΛΕΙΣΤΕΣ ...
                        ((aStar.isSelected()) ? true :
                              isInList(openSet,new Cell(r-1,c-1)) == -1 &&
                              isInList(closedSet,new Cell(r-1,c-1)) == -1)) {
                              Cell cell = new Cell(r-1,c-1);
                  
                        // ... ενημέρωσε τον δείκτη του πάνω-αριστερού κελιού να δείχνει το τρέχον ...
                        cell.prev = current;
                        // ... και πρόσθεσε το πάνω-αριστερό κελί στους διαδόχους του τρέχοντος. 
                        temp.add(cell);
                    
                }
            }
            // Επειδή στον αλγόριθμο DFS τα κελιά προστίθενται ένα-ένα στην
            // αρχή της λίστας ΑΝΟΙΚΤΕΣ, αντιστρέφουμε την σειρά των διαδόχων
            // που σχηματίστηκε, ώστε ο διάδοχος που αντιστοιχεί στην υψηλότερη
            // προτεραιότητα, να βρεθεί πρώτος στην λίστα.
            // Για τον, A*  δεν υπάρχει ζήτημα, γιατί η λίστα
            // ταξινομείται ως προς f ή dist πριν την εξαγωγή του πρώτου στοιχείου της.
            
            /*if (dfs.isSelected()){
                Collections.reverse(temp);
            }*/
            return temp;
        } // Τέλος της λειτουργίας createSuccesors()
        
        /**
         * Επιστρέφει τον δείκτη του κελιού current στη λίστα list
         *
         * @param list η λίστα μέσα στην οποία αναζητάμε
         * @param current το κελί που αναζητάμε
         * @return ο δείκτης το κελιού μέσα στη λίστα
         * αν το κελί δεν βρεθεί επιστρέφει -1
         */
        private int isInList(ArrayList<Cell> list, Cell current){
            int index = -1;
            for (int i = 0 ; i < list.size(); i++) {
                if (current.row == list.get(i).row && current.col == list.get(i).col) {
                    index = i;
                    break;
                }
            }
            return index;
        } // Τέλος της λειτουργίας isInList()
        
        /**
         * Επιστρέφει το προκάτοχο κελί του κελιού current της λίστας list
         *
         * @param list η λίστα μέσα στην οποία αναζητάμε
         * @param current το κελί που αναζητάμε
         * @return το κελί που αντιστοιχεί στον προκάτοχο του current
         */
        private Cell findPrev(ArrayList<Cell> list, Cell current){
            int index = isInList(list, current);
            return list.get(index).prev;
        } // Τέλος της λειτουργίας findPrev()
        
               
        /**
         * Υπολογίζει την διαδρομή από τον στόχο προς την αρχική θέση
         * του ρομπότ και μετρά τα αντίστοιχα βήματα
         * και την απόσταση που διανύθηκε.
         */
        private void plotRoute(){ //υπολογίζει από το τέλος
            searching = false;
            endOfSearch = true;
            int steps = 0;
            double distance = 0;
            int index = isInList(closedSet,targetPos); //ψάξε μέσα στην κλειστη λίστα για να δεις εαν υπάρχει ο τελικός προορισμός(targetPos)
            Cell cur = closedSet.get(index); //πάρε το targetPos(τις συντεταγμένες του τελικού προορισμού)
            grid[cur.row][cur.col]= TARGET;
            do {
                steps++;
                if (diagonal.isSelected()) 
                {
                    int dx = cur.col-cur.prev.col; //διαφορά τελικού στόχου απο τον προηγούμενο
                    int dy = cur.row-cur.prev.row;
                    //distance += Math.sqrt(dx*dx + dy*dy); //εαν κάνει διαγώνια κίνηση να υπολογίσεις την υποτείνουσα(στο δικό μας παράδειγμα θέλουμε σταθερο αριθμο το 2)
                    if(Math.abs(dx) == 1 && Math.abs(dy) == 1) //εαν η απόσταση των x και y είναι 1(δηλαδή όταν έχουμε διαγώνιο κελι...)
                    {
                        //distance += 2; //το κόστος μετάβασης θα είναι 2
                        distance += (int)((double)2*Math.sqrt(dx*dx + dy*dy));
                    }
                    else //εαν η απόσταση χ ή η απόσταση y δεν είναι 1 (εαν δεν έχουμε δηλαδή διαγώνιο κελί)
                    {
                        distance+= (Math.abs(dx)+Math.abs(dy)); //το κόστος μετάβασης θα είναι 1 
                    }         
                } else //εαν δεν θέλουμε διαγώνιο κίνηση 
                { 
                    distance++;  //η μετακινηση για το κάθε κελι θα είναι πάντα +1
                }
                cur = cur.prev;  //στο αντικείμενο prev έχουν αποθηκευτεί ΟΛΕΣ οι συντεταγμένες των προηγουμενων κόμβων(ΟΧΙ ομως αυτες που καταλήγαν σε αδιέξοδο)
                grid[cur.row][cur.col] = ROUTE;
               
                
            } while (!(cur.row == robotStart.row && cur.col == robotStart.col));
            grid[robotStart.row][robotStart.col]=ROBOT;
            String msg;
            msg = String.format("Κόμβοι που Εξερευνήθηκαν: %d, Βήματα: %d, Κόστος Μετάβασης: %.2f",
                     expanded,steps,distance); 
            message.setText(msg);
          
        } // Τέλος της λειτουργίας plotRoute()
        
        /**
         * Δίνει αρχικές τιμές στα κελιά του πλέγματος
         * Με το πρώτο κλικ στο κουμπί 'Καθάρισμα' μηδενίζει τα στοιχεία
         * της τυχόν αναζήτησης που είχε εκτελεστεί (Μέτωπο, Κλειστές, Διαδρομή) 
         * και αφήνει ανέπαφα τα εμπόδια και τις θέσεις ρομπότ και στόχου
         * προκειμένου να είναι δυνατή η εκτέλεση άλλου αλγόριθμου
         * με τα ίδια δεδομένα.
         * Με το δεύτερο κλικ αφαιρεί και τα εμπόδια.
         */
        private void fillGrid() {
            if (searching || endOfSearch) //όταν η αναζήτηση είναι ακόμα σε εξέληξη η έχουμε φτάσει στο τέλος με(με τα κελιά χρωματιστά)
            { 
                for (int r = 0; r < rows; r++) 
                {
                    for (int c = 0; c < columns; c++) {
                        if (grid[r][c] == FRONTIER || grid[r][c] == CLOSED || grid[r][c] == ROUTE) {
                            grid[r][c] = EMPTY;
                        }
                        if (grid[r][c] == ROBOT){ //εαν οι συντεταγμενες του κελιού ειναι αυτες του ΡΟΜΠΟΤ
                            robotStart = new Cell(r,c); //δημιούργησε εναν καινούριο Constructor με αυτες τις συντεταγμένες και βάλτες στο robotstart
                        }
                        if (grid[r][c] == TARGET){
                            targetPos = new Cell(r,c);
                        }
                    }
                }
               
                searching = false;
            } else //όταν δεν υπάρχει αναζήτηση
            {
                for (int r = 0; r < rows; r++) 
                {
                    for (int c = 0; c < columns; c++) 
                    {
                        grid[r][c] = EMPTY;
                        
                        if (grid[r][c] == ROBOT){
                            robotStart = new Cell(r,c);
                        }
                        if (grid[r][c] == TARGET){
                            targetPos = new Cell(r,c);
                        }
                    }
                }                               
                        
                
            }
            if (aStar.isSelected())
            {
                robotStart.g = 0;  //αρχικοποιούμε τις μεταβλητές
                robotStart.h = 0;
                robotStart.f = 0;
            }
            expanded = 0;
            found = false;
            searching = false;
            endOfSearch = false;
         
            // Το πρώτο βήμα των αλγόριθμων BFS και DFS γίνεται εδώ
            // 1. ΑΝΟΙΚΤΕΣ:= [So], ΚΛΕΙΣΤΕΣ:= []
            openSet.removeAll(openSet);
            openSet.add(robotStart); //βάλε στις ανοιχτες καταστάσεις την θέση του ΡΟΜΠΟΤ 
            closedSet.removeAll(closedSet);
            array.removeAll(array);
         
            grid[targetPos.row][targetPos.col] = TARGET; 
            grid[robotStart.row][robotStart.col] = ROBOT;
            message.setText(msgDrawAndSelect);
            timer.stop();
            repaint();
            
        } // Τέλος της λειτουργίας fillGrid()
        
        /**Extra μεθοδος - με το που ξεκινάει το πρόγραμμα να ανοίγει ο λαβύρινθος που έχουμε κατασκεύασει εμείς για τις ανάγκες της εργασίας */
        private void fillGrid1() {
            if (searching || endOfSearch) //όταν η αναζήτηση είναι ακόμα σε εξέληξη η έχουμε φτάσει στο τέλος με(με τα κελιά χρωματιστά)
            { 
                for (int r = 0; r < rows; r++) 
                {
                    for (int c = 0; c < columns; c++) {
                        if (grid[r][c] == FRONTIER || grid[r][c] == CLOSED || grid[r][c] == ROUTE) {
                            grid[r][c] = EMPTY;
                        }
                        if (grid[r][c] == ROBOT){
                            robotStart = new Cell(r,c);
                        }
                        if (grid[r][c] == TARGET){
                            targetPos = new Cell(r,c);
                        }
                    }
                }
               
                searching = false;
            } else //όταν δεν υπάρχει αναζήτηση
            {
                for (int r = 0; r < rows; r++) 
                {
                    for (int c = 0; c < columns; c++) 
                    {
                        grid[r][c] = EMPTY;
                    }
                }
                robotStart = new Cell(rows - 2,0);   //η αρχική θέση του ρομπότ
                targetPos = new Cell(rows - 2,columns-2);   //η αρχική θέση του προορισμού
                
             //δημιουργούμε μαύρα κελιά(εμπόδια) σύμφωνα με την εργασία του Παναγιωτόπουλου
                grid[0][columns -2] = OBST;
                grid[rows - 1][1] = OBST;
                grid[rows - 1][2] = OBST;
                grid[2][1] = OBST;
                grid[2][2] = OBST;
                grid[3][2] = OBST;            
                
            }
            if (aStar.isSelected()){
                robotStart.g = 0;  //αρχικοποιύμε τις μεταβλητές
                robotStart.h = 0;
                robotStart.f = 0;
            }
            expanded = 0;
            found = false;
            searching = false;
            endOfSearch = false;
         
            // Το πρώτο βήμα των υπόλοιπων αλγόριθμων γίνεται εδώ
            // 1. ΑΝΟΙΚΤΕΣ:= [So], ΚΛΕΙΣΤΕΣ:= []
            openSet.removeAll(openSet);
            openSet.add(robotStart);
            closedSet.removeAll(closedSet);
            array.removeAll(array);
         
            grid[targetPos.row][targetPos.col] = TARGET; 
            grid[robotStart.row][robotStart.col] = ROBOT;
            message.setText(msgDrawAndSelect);
            timer.stop();
            repaint();
            
        } // Τέλος της λειτουργίας fillGrid()
        

        /**
         * Ζωγραφίζει το πλέγμα
         */
        @Override
        public void paintComponent(Graphics g) 
        {

            super.paintComponent(g);  // Γεμίζει το background χρώμα.
             num_DFS = 0;

            g.setColor(Color.DARK_GRAY); //το χρωματικό πλαίσιο των ορθογωνίων 
            g.fillRect(10, 10, columns*squareSize+1, rows*squareSize+1);

            for (int r = 0; r < rows; r++) 
            {
                for (int c = 0; c < columns; c++) 
                {
                    switch (grid[r][c]) {
                        case EMPTY:
                            g.setColor(Color.WHITE);
                            break;
                        case ROBOT:
                            g.setColor(Color.GREEN);
                            break;
                        case TARGET:
                            g.setColor(Color.RED);
                            break;
                        case OBST:
                            g.setColor(Color.BLACK);
                            break;
                        case FRONTIER:
                            g.setColor(Color.BLUE);
                            break;
                        case CLOSED:
                            g.setColor(Color.CYAN);
                            break;
                        case ROUTE:
                            g.setColor(Color.YELLOW);
                            break;
                        default:
                            break;
                    }
                    g.fillRect(11 + c*squareSize, 11 + r*squareSize, squareSize - 1, squareSize - 1); //Γέμισμα των ορθογωνίων με χρώμα (αποσταση απο το χ, αποσταση απο ψ, μηκος πλατος τετραγωνου)
                }                            
            }           
            
            if (drawArrows.isSelected()) 
            {
                // Ζωγραφίζουμε όλα τα βέλη από κάθε ανοικτή ή κλειστή κατάσταση
                // προς την προκάτοχό της.
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < columns; c++) {
                        // Αν το τρέχον κελί είναι ο στόχος και έχει βρεθεί λύση
                        // ή είναι κελί της διαδρομής προς στο στόχο
                        // ή είναι ανοικτή κατάσταση,
                        // ή κλειστή αλλά όχι η αρχική θέση του ρομπότ
                        if ((grid[r][c] == TARGET && found)  || grid[r][c] == ROUTE  || 
                                grid[r][c] == FRONTIER || (grid[r][c] == CLOSED &&
                                !(r == robotStart.row && c == robotStart.col))){
                            // Η ουρά του βέλους είναι το τρέχον κελί, ενώ
                            // η κορυφή του βέλους είναι το προκάτοχο κελί.
                            Cell head;
                            if (grid[r][c] == FRONTIER)
                            {                                
                                head = findPrev(openSet,new Cell(r,c));
                                
                            } else 
                            {
                                head = findPrev(closedSet,new Cell(r,c));
                            }
                            // Οι συντεταγμένες του κέντρου του τρέχοντος κελιού
                            int tailX = 11+c*squareSize+squareSize/2;
                            int tailY = 11+r*squareSize+squareSize/2;
                            // Οι συντεταγμένες του κέντρου του προκάτοχου κελιού
                            int headX = 11+head.col*squareSize+squareSize/2;
                            int headY = 11+head.row*squareSize+squareSize/2;
                            // Αν το τρέχον κελί είναι ο στόχος
                            // ή είναι κελί της διαδρομής προς το στόχο ...
                            if (grid[r][c] == TARGET  || grid[r][c] == ROUTE){
                                // ... σχεδίασε ένα μπλέ βέλος προς την κατεύθυνση του στόχου.
                                g.setColor(Color.BLUE);
                                drawArrow(g,tailX,tailY,headX,headY);
                            // Αλλιώς ...
                            } else {
                                // ... σχεδίασε ένα μαύρο βέλος προς το προκάτοχο κελί.
                                g.setColor(Color.BLACK);
                                drawArrow(g,headX,headY,tailX,tailY);
                            }
                        }
                    }
                }
            }
            
            if(drawNumbers.isSelected())
            {
                  num_DFS = -1;                     
                for(Cell index : array)
                {                        
                    num_DFS++;
                    g.setColor(Color.blue); 
                    int fontSize = 1* squareSize/4;
                    g.setFont(new Font("Arial", Font.PLAIN, fontSize));   
                    g.drawString(String.valueOf(num_DFS), 11 + index.col*squareSize+squareSize/2, 11 + index.row*squareSize+squareSize/2);                                                    
                }   
            }
            
        } // Τέλος της λειτουργίας paintComponent()
        
        /**
         * Ζωγραφίζει ένα βέλος από το σημείο (x2,y2) προς το σημείο (x1,y1)
         */
        private void drawArrow(Graphics g1, int x1, int y1, int x2, int y2) {
            Graphics2D g = (Graphics2D) g1.create();

            double dx = x2 - x1, dy = y2 - y1;
            double angle = Math.atan2(dy, dx);
            int len = (int) Math.sqrt(dx*dx + dy*dy);
            AffineTransform at = AffineTransform.getTranslateInstance(x1, y1);
            at.concatenate(AffineTransform.getRotateInstance(angle));
            g.transform(at);

            // Εμείς ζωγραφίζουμε ένα οριζόντιο βέλος μήκους len
            // που καταλήγει στο σημείο (0,0) με τις δύο αιχμές μήκους arrowSize
            // να σχηματίζουν γωνίες 20 μοιρών με τον άξονα του βέλους ...
            g.drawLine(0, 0, len, 0);
            g.drawLine(0, 0, (int)(arrowSize*Math.sin(70*Math.PI/180)) , (int)(arrowSize*Math.cos(70*Math.PI/180)));
            g.drawLine(0, 0, (int)(arrowSize*Math.sin(70*Math.PI/180)) , -(int)(arrowSize*Math.cos(70*Math.PI/180)));
            // ... και η κλάση AffineTransform αναλαμβάνει τα υπόλοιπα !!!!!!
            // Πώς να μην θαυμάσει κανείς αυτήν την Java !!!!
        } // Τέλος της λειτουργίας drawArrow()
        
    } // Τέλος της εμφωλευμένης κλάσης MazePanel
  
} // Τέλος της κλάσης Maze