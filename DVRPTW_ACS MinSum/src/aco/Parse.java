package aco;

import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.cli.BasicParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;

/**
 * ACO algorithms for the TSP
 * 
 * This code is based on the ACOTSP project of Thomas Stuetzle.
 * It was initially ported from C to Java by Adrian Wilke.
 * 
 * Project website: http://adibaba.github.io/ACOTSPJava/
 * Source code: https://github.com/adibaba/ACOTSPJava/
 */
public class Parse {
	/***************************************************************************
     * Program's name: ACOTSPJava
     * 
     * Command line parser for 'ACO algorithms for the TSP'
     * 
     * Copyright (C) 2014 Adrian Wilke
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
     * You should have received a copy of the GNU General Public License along
     * with this program; if not, write to the Free Software Foundation, Inc.,
     * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
     ***************************************************************************/
	
    static class OptComparator implements Comparator<Option> {

		Map<String, Integer> opt = new HashMap<String, Integer>();
	
		public OptComparator() {
		    int i = 0;
		    
		    opt.put("u", i++);
		    opt.put("z", i++);
		}
	
		@Override
		public int compare(Option o1, Option o2) {
		    if (o1.getValue() == null || o2.getValue() == null) return 0;
		    else
		    	return (opt.get(o1.getOpt()) - opt.get(o2.getOpt()));
		}
    }

    static void parse_commandline(String args[], int runNumber) {
		if (args.length == 0) {
		    System.err.println("No options are specified.");
		    System.err.println("Try `--help' for more information.");
		    System.exit(1);
		}
	
		Options options = new Options();
		options.addOption("u", "as", false, "apply basic Ant System");
		options.addOption("z", "acs", false, "apply ant colony colony system");
	
		CommandLine cmd = null;
		CommandLineParser parser = new BasicParser();
		try {
		    cmd = parser.parse(options, args);
		} catch (ParseException e) {
		    System.err.println("Error: " + e.getMessage());
		    System.exit(1);
		}
	
		// Choice of ONE algorithm
		int algorithmCount = 0;
		if (cmd.hasOption("u")) {
		    algorithmCount++;
		}
		if (cmd.hasOption("z")) {
		    algorithmCount++;
		}
		if (algorithmCount > 1) {
		    System.err.println("Error: More than one ACO algorithm enabled in the command line.");
		    System.exit(1);
		} else if (algorithmCount == 1) {
		    Ants.as_flag = false;
		    Ants.acs_flag = false;
		}
	
		if (cmd.hasOption("u")) {
		    Ants.as_flag = true;
		    InOut.set_default_as_parameters();
		    System.out.println("\nRun basic Ant System #" + (runNumber + 1));
		}
		if (cmd.hasOption("z")) {
		    Ants.acs_flag = true;
		    InOut.set_default_acs_parameters();
		    System.out.println("\nRun Ant Colony System #" + (runNumber + 1));
		}

    }
}
