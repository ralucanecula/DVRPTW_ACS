package aco;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import aco.VRPTW_ACS;
import aco.Ants.Ant;

import org.apache.poi.ss.usermodel.Cell;
import org.apache.poi.ss.usermodel.CellStyle;
import org.apache.poi.ss.usermodel.Font;
import org.apache.poi.ss.usermodel.IndexedColors;
import org.apache.poi.ss.usermodel.Row;
import org.apache.poi.xssf.usermodel.XSSFSheet;
import org.apache.poi.xssf.usermodel.XSSFWorkbook;

/**
 * ACO algorithms for the TSP
 * 
 * This code is based on the ACOTSP project of Thomas Stuetzle.
 * It was initially ported from C to Java by Adrian Wilke.
 * 
 * Project website: http://adibaba.github.io/ACOTSPJava/
 * Source code: https://github.com/adibaba/ACOTSPJava/
 */
public class Utilities {
	/*
     * ################################################
     * ########## ACO algorithms for the TSP ##########
     * ################################################
     * 
     * Version: 1.0
     * File: utilities.c
     * Author: Thomas Stuetzle
     * Purpose: some additional useful procedures
     * Check: README and gpl.txt
     * Copyright (C) 2002 Thomas Stuetzle
     */

    /***************************************************************************
     * Program's name: acotsp
     * 
     * Ant Colony Optimization algorithms (AS, ACS, EAS, RAS, MMAS, BWAS) for the
     * symmetric TSP
     * 
     * Copyright (C) 2004 Thomas Stuetzle
     * 
     * This program is free software; you can redistribute it and/or modify
     * it under the terms of the GNU General Public License as published by
     * the Free Software Foundation; either version 2 of the License, or
     * (at your option) any later version.
     * 
     * This program is distributed in the hope that it will be useful,
     * but WITHOUT ANY WARRANTY; without even the implied warranty of
     * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
     * GNU General Public License for more details.
     * 
     * You should have received a copy of the GNU General Public License
     * along with this program; if not, write to the Free Software
     * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
     * 
     * email: stuetzle no@spam informatik.tu-darmstadt.de
     * mail address: Universitaet Darmstadt
     * Fachbereich Informatik
     * Hochschulstr. 10
     * D-64283 Darmstadt
     * Germany
     ***************************************************************************/
	
    private static Random random;

    static int seed;
    static int rowIndex = 1;
    
    //private static String filePath = "output/Experiments Fuzzy cMeans.xlsx";
  
    private static String filePath = "../../VRP/Experimente multi-objective VRPTW/Rulari multi-objective VRPTW - MoACO_D-ACS.xlsx";
    //private static String filePath1 = "../../../Jurnale/jurnal Swarm Intelligence (Springer)/Experimente/Fronturi Pareto algoritmi (pe parcurs)/ACO MinMax_vers. noua/ParetoFront_" + TSP_ACO.instanceName + " (m=" + MTsp.m + ")_amplitude_";
    private static String filePath1 = "../../VRP/Experimente multi-objective VRPTW/ParetoFront VRPTW_" + VRPTW.instance.name + " (MoACO_D-ACS)" + ".xlsx";
    private static String filePath2 = "../../VRP/Experimente multi-objective VRPTW/fisiere intermediare/Rulari MoACO_D-ACS_" + VRPTW.instance.name + "_no of vehicles and total distance.txt";
    private static String filePath3 = "../../Doctorat/VRP/VRPTW Solomon instances - data points.xlsx";
    //private static String filePath4 = "../../VRP/Experimente dynamic VRPTW Solomon/Rulari DVRPTW_ACS MinSum/";
    //private static String filePath4 = "output/";
    private static String filePath4 = "output_count_solutions/";
    //private static String filePath5 = "../../VRP/Experimente dynamic VRPTW Solomon/Rulari DVRPTW_ACS MinSum/Rulari DVRPTW_ACS MinSum.xlsx";
    //private static String filePath5 = "output/Rulari DVRPTW_ACS MinSum.xlsx";
    private static String filePath5 = "output_count_solutions/Rulari DVRPTW_ACS MinSum_instante statice_30 rulari.xlsx";
    
    //for setting the content to be inserted in the Excel file
    private static String tours[];
    private static int nrCities[];
    private static double subtoursCost[];
    private static double totalCost;
    private static double longestSubtour;
    private static double amplitude;
    
    //for verbose output: save at each 5 iteration the best (minimum) total cost of all m subtours 
    private static ArrayList<Double> iterTotalCost;
    
    //for verbose output: save at each 5 iteration the cost of the longest tour and the number of the corresponding iteration
    private static ArrayList<Double> iterLongestCost;
    private static ArrayList<Integer> iterNumber;
    
    //for displaying text with color at console of Linux based systems
    public static final String ANSI_RESET = "\u001B[0m";
    public static final String ANSI_RED = "\u001B[31m";

    //auxiliary routine for sorting an integer array
    static void swap2(double v[], int v2[], int i, int j) {
    	double tmp1;
		int tmp2;
	
		tmp1 = v[i];
		v[i] = v[j];
		v[j] = tmp1;
		
		tmp2 = v2[i];
		v2[i] = v2[j];
		v2[j] = tmp2;
    }

    //recursive routine (quicksort) for sorting one array; second array does the same sequence of swaps
    static void sort2(double v[], int v2[], int left, int right) {
		    int k, last;
		
			if (left >= right)
			    return;
			swap2(v, v2, left, (left + right) / 2);
			last = left;
			for (k = left + 1; k <= right; k++)
			    if (v[k] < v[left])
			    	swap2(v, v2, ++last, k);
			swap2(v, v2, left, last);
			sort2(v, v2, left, last);
			sort2(v, v2, last + 1, right);
    }
    
    //auxiliary routine for sorting an integer array and a double array
    static void swap2_(Double v[], Integer v2[], int i, int j) {
    	double tmp1;
		int tmp2;
	
		tmp1 = v[i];
		v[i] = v[j];
		v[j] = tmp1;
		
		tmp2 = v2[i];
		v2[i] = v2[j];
		v2[j] = tmp2;
    }

    //recursive routine (quicksort) for sorting one array; second array does the same sequence of swaps
    static void sort2_(Double v[], Integer v2[], int left, int right) {
		    int k, last;
		
			if (left >= right)
			    return;
			swap2_(v, v2, left, (left + right) / 2);
			last = left;
			for (k = left + 1; k <= right; k++)
			    if (v[k] < v[left])
			    	swap2_(v, v2, ++last, k);
			swap2_(v, v2, left, last);
			sort2_(v, v2, left, last);
			sort2_(v, v2, last + 1, right);
    }

    //generate a random number that is uniformly distributed in [0,1]
    static double random01() {
		if (random == null) {
		    random = new Random();
		}
	
		return random.nextDouble();
    }
    
    static void writeInputDataPoints() {
    	//the file already exists
    	if (new File(filePath3).canRead()) {
	    	//System.out.println("File already exists..");
	    	try {
		    	FileInputStream file = new FileInputStream(new File(filePath3));
		    	
		    	//Create Workbook instance holding reference to .xlsx file
	            XSSFWorkbook workbook1 = new XSSFWorkbook(file);
	 
	            //Get first/desired sheet from the workbook
	            XSSFSheet sheet1 = workbook1.getSheetAt(11);
		    	
	            //define a cell style for bold font
	            CellStyle style = workbook1.createCellStyle();
	            Font font = workbook1.createFont();
	            font.setBoldweight(Font.BOLDWEIGHT_BOLD);
	            style.setFont(font);
	            
	            Row r1 = sheet1.getRow(0); 
	            if (r1 == null) {
	               // First cell in the row, create
	               r1 = sheet1.createRow(0);
	            }

	            Cell c = r1.getCell(0); 
	            if (c == null) {
	                c = r1.createCell(0);
	            }
	            c.setCellValue("VRPTW instance - " + VRPTW.instance.name + "; data point coordinates corresponding to customers' requests");
	            c.setCellStyle(style);
	            
	            Row r = sheet1.getRow(2); 
	            if (r == null) {
	               // First cell in the row, create
	               r = sheet1.createRow(2);
	            }

	            Cell c1 = r.getCell(0); 
	            if (c1 == null) {
	                c1 = r.createCell(0);
	            }
	            c1.setCellValue("Point #");
	            c1.setCellStyle(style);
	            
	            c1 = r.getCell(1); 
	            if (c1 == null) {
	                c1 = r.createCell(1);
	            }
	            c1.setCellValue("X Coord");
	            c1.setCellStyle(style);
	            
	            c1 = r.getCell(2); 
	            if (c1 == null) {
	                c1 = r.createCell(2);
	            }
	            c1.setCellValue("Y Coord");
	            c1.setCellStyle(style);
	            
	            int size = VRPTW.instance.nodes.length;
	            int rowIndex = 3;
	            double x, y;
	            for (int i = 0; i < size; i++) {
	            	x = VRPTW.instance.nodes[i].x;
	            	y = VRPTW.instance.nodes[i].y;
	            	r = sheet1.getRow(rowIndex + i); 
		            if (r == null) {
		               // First cell in the row, create
		               //System.out.println("Empty row, create new one");
		               r = sheet1.createRow(rowIndex + i);
		            }
	
		            c1 = r.getCell(0); 
		            if (c1 == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c1 = r.createCell(0);
		            }
		            c1.setCellValue(i);
            	    
            	    c1 = r.getCell(1); 
		            if (c1 == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c1 = r.createCell(1);
		            }
		            c1.setCellValue(x);
		            
		            c1 = r.getCell(2); 
		            if (c1 == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c1 = r.createCell(2);
		            }
		            c1.setCellValue(y);
            	    
	            }
	              
				//Write the workbook in file system
			    FileOutputStream out = new FileOutputStream(new File(filePath3));
			    workbook1.write(out);
			    out.close();
			    
			    //System.out.println("Written successfully on disk.");
	    	}
			catch (Exception e) {
			    e.printStackTrace();
			}

	    }
    	else {
    		System.out.println("File not exists..");
    	}
    }
    
    static void writeExcel(int n, int m, int result) {
    	//the file already exists; we should add a new row as the last one in the Excel file
	    if (new File(filePath).canRead()) {
	    	//System.out.println("File already exists..");
	    	try {
		    	FileInputStream file = new FileInputStream(new File(filePath));
		    	
		    	//Create Workbook instance holding reference to .xlsx file
	            XSSFWorkbook workbook1 = new XSSFWorkbook(file);
	 
	            //Get first/desired sheet from the workbook
	            XSSFSheet sheet1 = workbook1.getSheetAt(2);
		    	int countRows = sheet1.getLastRowNum() + 1;
		    	Row newRow = sheet1.createRow(countRows++);
		    	
		    	int cellnum = 0;
		    	Cell cell = newRow.createCell(cellnum++);
		    	cell.setCellValue(n);
		    	cell = newRow.createCell(cellnum++);
		    	cell.setCellValue(m);
		    	cell = newRow.createCell(cellnum++);
		    	cell.setCellValue(result);
			    
				//Write the workbook in file system
			    FileOutputStream out = new FileOutputStream(new File(filePath));
			    workbook1.write(out);
			    out.close();
			    
			    //System.out.println("Written successfully on disk.");
	    	}
			catch (Exception e) {
			    e.printStackTrace();
			}

	    }
	    else {
	    	//Blank workbook
			XSSFWorkbook workbook2 = new XSSFWorkbook(); 
			
			//Create a blank sheet
			XSSFSheet sheet2 = workbook2.createSheet("Results - 51 cities");
		 
			//Iterate over data and write to sheet
			int rownum = 0, cellnum = 0;
			Row row = sheet2.createRow(rownum++);
			Cell cell = row.createCell(cellnum++);
			cell.setCellValue(n);
	    	cell = row.createCell(cellnum++);
	    	cell.setCellValue(m);
	    	cell = row.createCell(cellnum++);
	    	cell.setCellValue(result);
			
			try {
				//Write the workbook in file system
			    FileOutputStream out = new FileOutputStream(new File(filePath));
			    workbook2.write(out);
			    out.close();
			    
			    //System.out.println("Written successfully on disk.");
			} 
			catch (Exception e) {
			    e.printStackTrace();
			}
			    
	    }
    }
    
    static void writeResultsExcel(int trialNumber, boolean saveIterCosts) {
    	Row r, r1;
    	Cell c;
    	int index1 = 0, index2 = 0, index3 = 0, index4 = 0, index5 = 0;
    	//int index6 = 0;
    	
    	//the file already exists; we should add a new row as the last one in the Excel file
	    if (new File(filePath).canRead()) {
	    	//System.out.println("File already exists..");
	    	try {
		    	FileInputStream file = new FileInputStream(new File(filePath));
		    	
		    	//Create Workbook instance holding reference to .xlsx file
	            XSSFWorkbook workbook1 = new XSSFWorkbook(file);
	            
	            int startIndex = 0, rowIndex = 0;
	            /*switch (VRPTW.m) {
	            	case 2: 
	            		startIndex = 0;
	            		rowIndex = 4;
	            		break;
	            	case 3: 
	            		startIndex = 2;
	            		rowIndex = 5;
	            		break;
	            	case 5: 
	            		startIndex = 4;
	            		rowIndex = 7;
	            		break;
	            	case 7: 
	            		startIndex = 6;
	            		rowIndex = 9;
	            		break;
	            	default:
	            		System.out.println("Unknown value for m");
	            		break;         
	            }*/
	            
	            //Get desired sheet from the workbook
	            XSSFSheet sheet1 = workbook1.getSheetAt(startIndex);  //for tours
	            /*XSSFSheet sheet2 = workbook1.getSheetAt(startIndex + 1);  //for number of assigned cities
	            XSSFSheet sheet3 = workbook1.getSheetAt(startIndex + 2);  //for cost of individual subtours
	            XSSFSheet sheet4 = workbook1.getSheetAt(startIndex + 3);  //for total cost of subtours
	            XSSFSheet sheet5 = workbook1.getSheetAt(startIndex + 4);  //for verbose output of total cost at each 5 iteration
	            */
	            XSSFSheet sheet2 = workbook1.getSheetAt(startIndex + 1);  //for verbose output of longest cost at each 5 iteration
	            
	            //define a cell style for bold font
	            CellStyle style = workbook1.createCellStyle();
	            Font font = workbook1.createFont();
	            font.setBoldweight(Font.BOLDWEIGHT_BOLD);
	            style.setFont(font);
	            
	            //define style with bold font and blue color for font
	            CellStyle styleBoldBlue = workbook1.createCellStyle();
	            font = workbook1.createFont();
	            font.setBoldweight(Font.BOLDWEIGHT_BOLD);
	            font.setColor(IndexedColors.BLUE.index);
	            styleBoldBlue.setFont(font);
	            
	            index1 = 133;
	            if (!saveIterCosts) {
	            	//write only once the name of the algorithm that was run
	            	if (trialNumber == 0) {
			            r = sheet1.getRow(index1); 
			            if (r == null) {
			               // First cell in the row, create
			               //System.out.println("Empty row, create new one");
			               r = sheet1.createRow(index1);
			            }
		
			            c = r.getCell(0); 
			            if (c == null) {
			                // New cell
			            	//System.out.println("Empty cell, create new one");
			                c = r.createCell(0);
			            }
			            c.setCellValue("Obtained solutions (values) after running new version (ACS MinMax global, voiajor si oras alesi simultan) with local search");
			            c.setCellStyle(styleBoldBlue);
	            	}
            	
                    //write only once the table header
		            index1 = index1 + 3;
		            r = sheet1.getRow(index1); 
		            if (r == null) {
		               // First cell in the row, create
		               //System.out.println("Empty row, create new one");
		               r = sheet1.createRow(index1);
		            }
	
		            c = r.getCell(0); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(0);
		            }
		            c.setCellValue("Run #");
		            c.setCellStyle(style);
		            
		            c = r.getCell(1); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(1);
		            }
		            c.setCellValue("MinMax (cost of longest subtour)");
		            c.setCellStyle(style);
		            
		            c = r.getCell(2); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(2);
		            }
		            c.setCellValue("Total Cost");
		            c.setCellStyle(style);
		            
		            c = r.getCell(3); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(3);
		            }
		            c.setCellValue("Amplitude");
		            c.setCellStyle(style);
		            
		            //write number of run
		            index1 = 137 + trialNumber;
		            r = sheet1.getRow(index1); 
		            if (r == null) {
		               // First cell in the row, create
		               //System.out.println("Empty row, create new one");
		               r = sheet1.createRow(index1);
		            }
	
		            c = r.getCell(0); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(0);
		            }
		            c.setCellValue(trialNumber + 1);

		            //write MinMax (cost of longest subtour)
		            double longestSubtour = getLongestSubtour();
		            c = r.getCell(1); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(1);
		            }
		            c.setCellValue(longestSubtour); 
		            
		            //write total cost
		            double totalCost = getTotalCost();
		            c = r.getCell(2); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(2);
		            }
		            c.setCellValue(totalCost); 
		            
		            //write amplitude
		            double amplitude = getAmplitude();
		            c = r.getCell(3); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(3);
		            }
		            c.setCellValue(amplitude); 
            	}
	            
	            index5 = 859;
	            if (saveIterCosts) {
	            	//write only once the name of the algorithm that was run
	            	if (trialNumber == 0) {
			            r = sheet2.getRow(index5); 
			            if (r == null) {
			               // First cell in the row, create
			               //System.out.println("Empty row, create new one");
			               r = sheet2.createRow(index5);
			            }
		
			            c = r.getCell(0); 
			            if (c == null) {
			                // New cell
			            	//System.out.println("Empty cell, create new one");
			                c = r.createCell(0);
			            }
			            c.setCellValue("Longest cost of subtour at each 5 iteration after running new version (ACS MinMax global, voiajor si oras alesi simultan) with local search");
			            c.setCellStyle(styleBoldBlue);
			          
			            int tempIndex = index5 + 3;
			            r = sheet2.getRow(tempIndex);
			            if (r == null) {
				               // First cell in the row, create
				               //System.out.println("Empty row, create new one");
				               r = sheet2.createRow(tempIndex);
				        }
			            ArrayList<Integer> iterNumber = getIterNumber();
			            
			            c = r.getCell(0); 
			            if (c == null) {
			                // New cell
			            	//System.out.println("Empty cell, create new one");
			                c = r.createCell(0);
			            }
			            c.setCellValue("Nr Iter");
			            c.setCellStyle(style); 
			            
			            int indexTemp = 0;
			            for (int j = 0; j < iterNumber.size(); j++) {
			            	indexTemp = tempIndex + 1 + j;
				            r1 = sheet2.getRow(indexTemp); 
				            if (r1 == null) {
				               // First cell in the row, create
				               //System.out.println("Empty row, create new one");
				               r1 = sheet2.createRow(indexTemp);
				            }
			
				            c = r1.getCell(0); 
				            if (c == null) {
				                // New cell
				            	//System.out.println("Empty cell, create new one");
				                c = r1.createCell(0);
				            }
				            c.setCellValue(iterNumber.get(j));     
		            	}            
	            	}
	            	
	            	index5 = index5 + 3;
		            r = sheet2.getRow(index5); 
		            if (r == null) {
		               // First cell in the row, create
		               //System.out.println("Empty row, create new one");
		               r = sheet2.createRow(index5);
		            }
		            
		            //for each trial run save at each 5 iteration the best longest cost of a subtour so far
		            ArrayList<Double> iterLongestCost = getIterLongestCost();
		            int index;
		            
		            //for each run write the table header cell
		            c = r.getCell(trialNumber + 1); 
		            if (c == null) {
		                // New cell
		            	//System.out.println("Empty cell, create new one");
		                c = r.createCell(trialNumber + 1);
		            }
		            c.setCellValue("Run " + (trialNumber + 1));
		            c.setCellStyle(style); 
		            	
		            for (int j = 0; j < iterLongestCost.size(); j++) {
	            		index = index5 + 1 + j;
			            r1 = sheet2.getRow(index); 
			            if (r1 == null) {
			               // First cell in the row, create
			               //System.out.println("Empty row, create new one");
			               r1 = sheet2.createRow(index);
			            }
		
			            c = r1.getCell(trialNumber + 1); 
			            if (c == null) {
			                // New cell
			            	//System.out.println("Empty cell, create new one");
			                c = r1.createCell(trialNumber + 1);
			            }
			            c.setCellValue(iterLongestCost.get(j));     
	            	}
	            }

				//Write the workbook in file system
			    FileOutputStream out = new FileOutputStream(new File(filePath));
			    workbook1.write(out);
			    out.close();
			    
			    int nrOfRun = trialNumber + 1;
			    System.out.println("\nRun #" + nrOfRun + " written successfully on disk.\n");
	    	}
			catch (Exception e) {
			    e.printStackTrace();
			}

	    }
	    else {
	    	//Blank workbook
	    	System.out.println("File " + filePath + " doesn't exists.."); 
			    
	    }
    }
    
    static void writeParetoSet(ArrayList<Ant> bestSoFarPareto, int trial) {
    	Row r;
    	Cell c;
    	int lineNumber = 0;
    	
    	//filePath1 += InOut.max_iterations + " iter (ACO MinMax_vers noua).xlsx";
    	//System.out.println("file path=" + filePath1);
    	 
    	//the file already exists; we should add a new row as the last one in the Excel file
	    if (new File(filePath1).canRead()) {
	    	//System.out.println("File already exists..");
	    	try {
		    	FileInputStream file = new FileInputStream(new File(filePath1));
		    	
		    	//Create Workbook instance holding reference to .xlsx file
	            XSSFWorkbook workbook1 = new XSSFWorkbook(file);
	 
	            //Get first/desired sheet from the workbook
	            XSSFSheet sheet1 = workbook1.getSheetAt(trial);
	            
	            //write table header cells
	            r = sheet1.getRow(lineNumber); 
	            if (r == null) {
	               // First cell in the row, create
	               r = sheet1.createRow(lineNumber);
	            }
	            c = r.getCell(0); 
	            if (c == null) {
	                // New cell
	                c = r.createCell(0);
	            }
	            c.setCellValue("Point #");   
	            c = r.getCell(1); 
	            if (c == null) {
	                // New cell
	                c = r.createCell(1);
	            }
	            c.setCellValue("Total tours length");            
	            c = r.getCell(2); 
	            if (c == null) {
	                // New cell
	                c = r.createCell(2);
	            }
	            c.setCellValue("Amplitude of tours");	            
	            c = r.getCell(3); 
	            if (c == null) {
	                // New cell
	                c = r.createCell(3);
	            }
	            c.setCellValue("List with cost of subtours");	
	            
	            lineNumber++;
	            for (int i = 0; i < bestSoFarPareto.size(); i++) {
	            	r = sheet1.getRow(i + lineNumber); 
		            if (r == null) {
		               // First cell in the row, create
		               r = sheet1.createRow(i + lineNumber);
		            }
		            //write point id
		            c = r.getCell(0); 
		            if (c == null) {
		                // New cell
		                c = r.createCell(0, Cell.CELL_TYPE_NUMERIC);
		            }
		            c.setCellValue(i + 1);
		            //write total cost and amplitude
	            	for (int indexObj = 0; indexObj < 2; indexObj++) {
	            		c = r.getCell(indexObj + 1); 
			            if (c == null) {
			                // New cell
			                c = r.createCell(indexObj + 1, Cell.CELL_TYPE_NUMERIC);
			            }
			            c.setCellValue(bestSoFarPareto.get(i).costObjectives[indexObj]);
	            	}
	            	//write cost of each individual subtour
		            for (int j = 0; j < bestSoFarPareto.get(i).tour_lengths.size(); j++) {
			            c = r.getCell(j + 3); 
			            if (c == null) {
			                // New cell
			                c = r.createCell(j + 3);
			            }
			            c.setCellValue(bestSoFarPareto.get(i).tour_lengths.get(j));
		            }
	            }
    
				//Write the workbook in file system
			    FileOutputStream out = new FileOutputStream(new File(filePath1));
			    workbook1.write(out);
			    out.close();
			    
			    //System.out.println("\nWritten Pareto front points successfully on disk.\n");
			    int nrOfRun = trial + 1;
			    System.out.println("\nRun #" + nrOfRun + " written Pareto front points successfully on disk.\n");
	    	}
			catch (Exception e) {
			    e.printStackTrace();
			}

	    }
	    else {
	    	System.out.println(" File " + filePath1 + " doesn't exists" );
	    }

    }

    //save in a .txt output file the best solution resulted after a run to be later used when
    //computing the Pareto front
    static void writeParetoSolutions(ArrayList<Ant> bestSoFarPareto) {
    	File f = new File(filePath2);
    	double[] objValues = new double[2];
    	
    	try {
    		BufferedWriter bw = new BufferedWriter(new FileWriter(f, true));
    			
    		for (int i = 0; i < bestSoFarPareto.size(); i++) {
    			if (rowIndex > 0 && i != 0) {
    				bw.newLine();
    			}
    			bw.write(rowIndex + "\t");
    			//get values total cost and amplitude
    			for (int indexObj = 0; indexObj < 2; indexObj++) {
    				objValues[indexObj] = bestSoFarPareto.get(i).costObjectives[indexObj];
    			}
    			bw.write(objValues[0] + "\t");
    			//write cost of each individual subtour
    			for (int j = 0; j < bestSoFarPareto.get(i).tour_lengths.size(); j++) {
    				bw.write(bestSoFarPareto.get(i).tour_lengths.get(j) + "\t");
    			}
    			bw.write(objValues[1] + "\t");
    			
    			rowIndex++;
    		}
    		bw.newLine();
    		bw.close();
        }
    	catch (IOException e) {
    		System.out.println("error writing file");
        }
    }
    
    static void writeFinalSolution(int trial, String fileName, double scalledValue, boolean isValid) {
    	String name = filePath4 + fileName;
    	File f = new File(name);
    	
    	try {
    		BufferedWriter bw = new BufferedWriter(new FileWriter(f, true));

    		if (trial > 0) {
    			bw.newLine();
    		}
			bw.write("Run Ant Colony System #" + (trial + 1));
			
			bw.write("\nFinal best solution >> No. of used vehicles=" + Ants.best_so_far_ant.usedVehicles + " total tours length=" + Ants.best_so_far_ant.total_tour_length + " (scalled value = " + scalledValue + ")");
			bw.newLine();
			for (int i = 0; i < Ants.best_so_far_ant.usedVehicles; i++) {
				int tourLength = Ants.best_so_far_ant.tours.get(i).size();
				for (int j = 0; j < tourLength; j++) {
					int city = Ants.best_so_far_ant.tours.get(i).get(j);
					city = city + 1;  //so as to correspond to the city indexes from the VRPTW input file
					bw.write(city + " ");		
				}
				bw.newLine();
			}   
			bw.write("\nTotal number of evaluations: " + InOut.noEvaluations);
			bw.write("\nAdded nodes=" + Controller.addedNodes); 
		    if (isValid) {
		    	bw.write("\nThe final solution is valid (feasible)..");
		    }
		    else {
		    	bw.write("\nThe final solution is not valid (feasible)..");
		    }
    		
    		bw.newLine();
    		bw.close();
    		
        }
    	catch (IOException e) {
    		System.out.println("error writing file");
    		e.printStackTrace();
        }
    }
    
    static void writeExcelFinalSolution(int trial, double scalledValue) {
    	Row r;
    	Cell c;
    	int index1 = 0;
    	
    	//the file already exists; we should add a new row as the last one in the Excel file
	    if (new File(filePath5).canRead()) {
	    	//System.out.println("File already exists..");
	    	try {
		    	FileInputStream file = new FileInputStream(new File(filePath5));
		    	
		    	//Create Workbook instance holding reference to .xlsx file
	            XSSFWorkbook workbook1 = new XSSFWorkbook(file);
	            
	            //Get desired sheet from the workbook
	            XSSFSheet sheet1 = workbook1.getSheetAt(0); 
	            
	            //define a cell style for bold font
	            CellStyle style = workbook1.createCellStyle();
	            Font font = workbook1.createFont();
	            font.setBoldweight(Font.BOLDWEIGHT_BOLD);
	            style.setFont(font);
	            
	            //define style with bold font and blue color for font
	            CellStyle styleBoldBlue = workbook1.createCellStyle();
	            font = workbook1.createFont();
	            font.setBoldweight(Font.BOLDWEIGHT_BOLD);
	            font.setColor(IndexedColors.BLUE.index);
	            styleBoldBlue.setFont(font);
	            
	            index1 = 8;   //8  //26
	            
	            index1 = index1 + trial;
	            r = sheet1.getRow(index1); 
	            if (r == null) {
	               // First cell in the row, create
	               //System.out.println("Empty row, create new one");
	               r = sheet1.createRow(index1);
	            }

	            int nrOfRun = trial + 1;
	            //write trial number (Run #)
	            c = r.getCell(15); 
	            if (c == null) {
	                // New cell
	            	//System.out.println("Empty cell, create new one");
	                c = r.createCell(15);
	            }
	            c.setCellValue(nrOfRun);
	            
	            //write number of used vehicles
	            c = r.getCell(16); 
	            if (c == null) {
	                // New cell
	            	//System.out.println("Empty cell, create new one");
	                c = r.createCell(16);
	            }
	            c.setCellValue(Ants.best_so_far_ant.usedVehicles);

	            //write total traveled distance
	            c = r.getCell(17); 
	            if (c == null) {
	                // New cell
	            	//System.out.println("Empty cell, create new one");
	                c = r.createCell(17);
	            }
	            c.setCellValue(scalledValue); 
	            
	            //write the total number of feasible solutions
	            c = r.getCell(18); 
	            if (c == null) {
	                // New cell
	            	//System.out.println("Empty cell, create new one");
	                c = r.createCell(18);
	            }
	            c.setCellValue(InOut.noSolutions); 

				//Write the workbook in file system
			    FileOutputStream out = new FileOutputStream(new File(filePath5));
			    workbook1.write(out);
			    out.close();
			    
			    System.out.println("\nRun #" + nrOfRun + " written successfully on disk.\n");
	    	}
	    	
			catch (Exception e) {
			    e.printStackTrace();
			}

	    }
	    else {
	    	//Blank workbook
	    	System.out.println("File " + filePath5 + " doesn't exists.."); 
			    
	    }
    }
    

	public static String[] getTours() {
		return tours;
	}

	public static void setTours(String[] tours) {
		Utilities.tours = tours;
	}

	public static int[] getNrCities() {
		return nrCities;
	}

	public static void setNrCities(int[] nrCities) {
		Utilities.nrCities = nrCities;
	}

	public static double[] getSubtoursCost() {
		return subtoursCost;
	}

	public static void setSubtoursCost(double[] subtoursCost) {
		Utilities.subtoursCost = subtoursCost;
	}

	public static double getTotalCost() {
		return totalCost;
	}

	public static void setTotalCost(double totalCost) {
		Utilities.totalCost = totalCost;
	}

	public static ArrayList<Double> getIterTotalCost() {
		return iterTotalCost;
	}

	public static void setIterTotalCost(ArrayList<Double> iterTotalCost) {
		Utilities.iterTotalCost = iterTotalCost;
	}

	public static ArrayList<Double> getIterLongestCost() {
		return iterLongestCost;
	}

	public static ArrayList<Integer> getIterNumber() {
		return iterNumber;
	}

	public static void setIterLongestCost(ArrayList<Double> iterLongestCost) {
		Utilities.iterLongestCost = iterLongestCost;
	}
	
	public static void setIterNumber(ArrayList<Integer> iterNumber) {
		Utilities.iterNumber = iterNumber;
	}

	public static double getLongestSubtour() {
		return longestSubtour;
	}

	public static void setLongestSubtour(double longestSubtour) {
		Utilities.longestSubtour = longestSubtour;
	}

	public static double getAmplitude() {
		return amplitude;
	}

	public static void setAmplitude(double amplitude) {
		Utilities.amplitude = amplitude;
	}
	
	public static ArrayList<ArrayList<Integer>> readSolution(String fileName) {
		File file = new File("input/" + fileName);
		BufferedReader in = null;
		int value;
		
		ArrayList<ArrayList<Integer>> solution = new ArrayList();
		ArrayList<Integer> tour;
		
        if (file.exists()) {
        	try {
                in = new BufferedReader(new FileReader(file));
                String line = in.readLine();
                while (line != null) {
                	tour = new ArrayList<Integer>();
                	String[] strRecord = line.trim().split(" ");
                	for (int i = 0; i < strRecord.length; i++) {
                		try {
                            value = Integer.parseInt(strRecord[i]) - 1;
                            tour.add(value);
                        } catch (NumberFormatException e) {
                            System.out.println("NumberFormatException " + e.getMessage() + " record=" + strRecord[i] + " line=" + line);
                        }
                	}
                	solution.add(tour);
                	line = in.readLine();
                }
                in.close();
            } catch (FileNotFoundException ignored) {
            } catch (IOException e) {
                System.out.println("Error occurred while reading file: " + file + " " + e.getMessage());
            }
        	
        	
        }
        
        return solution;
		
	}
	
	public static boolean checkFeasibility(Ant a, VRPTW vrp, boolean printNoNodes) {
		boolean isFeasible = true;
		int currentCity, prevCity, addedNodes = 0;
		double currentQuantity, currentTime;
		double distance, arrivalTime, beginService;
		ArrayList<Request> reqList = vrp.getRequests();
		
		for (int indexTour = 0; indexTour < a.usedVehicles; indexTour++) {
			currentQuantity = reqList.get(0).getDemand();
			currentTime = 0.0;
			for (int currentPos = 1; currentPos < a.tours.get(indexTour).size(); currentPos++) {
				if (currentPos < a.tours.get(indexTour).size() - 1) {
					addedNodes++;
				}
				/*if (addedNodes == 0) {
					System.out.println("Possible empty tour: " + a.tours.get(indexTour).size());
				}*/
				prevCity = a.tours.get(indexTour).get(currentPos - 1);
				currentCity = a.tours.get(indexTour).get(currentPos);
				currentQuantity += reqList.get(currentCity + 1).getDemand();
				
				distance = VRPTW.instance.distance[prevCity + 1][currentCity + 1];
		    	arrivalTime = currentTime + reqList.get(prevCity + 1).getServiceTime() + distance; 
		    	beginService = Math.max(arrivalTime, reqList.get(currentCity + 1).getStartWindow());
		    	if (beginService > reqList.get(currentCity + 1).getEndWindow()) {
		    		//isFeasible = false;
		    		System.out.println("Time window constraint violated");
		    		return false;
		    	}
		    	currentTime = beginService;	
				
			}
			if (currentQuantity > vrp.getCapacity()) {
				//isFeasible = false;
				System.out.println("Capacity constraint violated");
				return false;
			}
		}
		if (printNoNodes) {
			System.out.println("Added nodes=" + addedNodes);
		}
		Controller.addedNodes = addedNodes;
		return isFeasible;
		
	}

	public static void main(String[] args) {
		
		//read the data from the input file
		DataReader reader = new DataReader(Controller.vrpInstance);
        //read the data from the file
        VRPTW vrpInstance = reader.read();
        int trial = 1;
        String solutionFile = "RC203_solutie_new.txt";
        
        System.out.println("\nVRPTW_ACS MinSum_alegere simultana >> Solving VRPTW instance: " + VRPTW.instance.name);
        System.out.println("No. of customers' requests (except the depot): " +  VRPTW.n);
        //System.out.println("No. of desired tours (used vehicles): " + VRPTW.m);
        
        InOut.init_program(args, trial, vrpInstance, 0.0);

		Ants.ants[0].tours = readSolution(solutionFile);
		boolean res = checkFeasibility(Ants.ants[0], vrpInstance, true);
		if (res) {
			System.out.println("Solution is feasible");
		}
		else {
			System.out.println("Solution is not feasible");
		}
		
	}

}
